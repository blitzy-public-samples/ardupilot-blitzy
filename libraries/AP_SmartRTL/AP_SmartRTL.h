/**
 * @file AP_SmartRTL.h
 * @brief Smart Return-to-Launch (RTL) path recording and optimization system
 * 
 * @details This library implements an intelligent return path system that records
 *          the vehicle's flight path as a series of waypoints (breadcrumbs) and
 *          generates an optimized return path when needed.
 *          
 *          Architecture:
 *          - Path Recording: Continuously saves vehicle position at configurable
 *            intervals (~3Hz) as it moves, storing points in NED frame relative
 *            to EKF origin
 *          - Path Simplification: Uses Ramer-Douglas-Peucker algorithm to remove
 *            redundant points that lie on approximately straight segments
 *          - Loop Pruning: Detects and removes loops in the path (e.g., when
 *            vehicle circles back on itself) to create shorter return route
 *          - Background Processing: Cleanup algorithms run in IO thread with
 *            time-slicing to avoid impacting flight controller performance
 *          
 *          Memory Management:
 *          - Configurable point limit (SMARTRTL_POINTS parameter, max 500)
 *          - Dynamic memory allocation at initialization (~20 bytes per point)
 *          - Automatic cleanup triggers when approaching memory limits
 *          - Deactivation if memory allocation fails or path becomes too complex
 *          
 *          Integration:
 *          - Called from vehicle code at ~3Hz via update() method
 *          - Returns optimized path points via pop_point() during RTL
 *          - Thread-safe operation with semaphore protection for path access
 *          - Coordinates with EKF for position data and home location
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger_config.h>

// definitions and macros

/**
 * Default minimum distance between path points in meters.
 * Points closer than this will be merged during path recording to save memory.
 */
#define SMARTRTL_ACCURACY_DEFAULT        2.0f   // default _ACCURACY parameter value.  Points will be no closer than this distance (in meters) together.

/**
 * Default maximum number of path points.
 * Memory usage: approximately 20 bytes per point.
 * Higher values allow longer paths but increase memory and CPU usage for cleanup algorithms.
 */
#define SMARTRTL_POINTS_DEFAULT          300    // default _POINTS parameter value.  High numbers improve path pruning but use more memory and CPU for cleanup. Memory used will be 20bytes * this number.

/**
 * Absolute maximum number of path points supported by this library.
 * This is a hard limit due to fixed-size bitmask and array allocations.
 * @warning Exceeding this value will cause deactivation of SmartRTL for the flight.
 */
#define SMARTRTL_POINTS_MAX              500    // the absolute maximum number of points this library can support.

/**
 * Timeout in milliseconds for path recording inactivity.
 * If no points are saved for this duration, SmartRTL is deactivated for the flight.
 * Typical causes: GPS loss, position estimate unavailable, or vehicle stationary.
 */
#define SMARTRTL_TIMEOUT                 15000  // the time in milliseconds with no points saved to the path (for whatever reason), before SmartRTL is disabled for the flight

/**
 * Number of new points added before triggering simplification algorithm.
 * Balances cleanup frequency with computational overhead.
 */
#define SMARTRTL_CLEANUP_POINT_TRIGGER   50     // simplification will trigger when this many points are added to the path

/**
 * Number of empty path slots remaining when routine cleanup begins.
 * Early cleanup prevents path from filling completely and requiring emergency measures.
 */
#define SMARTRTL_CLEANUP_START_MARGIN    10     // routine cleanup algorithms begin when the path array has only this many empty slots remaining

/**
 * Minimum number of points that must be removed for cleanup to be considered worthwhile.
 * Prevents wasting CPU cycles on cleanups that free negligible space.
 */
#define SMARTRTL_CLEANUP_POINT_MIN       10     // cleanup algorithms will remove points if they remove at least this many points

/**
 * Epsilon tolerance for Ramer-Douglas-Peucker simplification algorithm in meters.
 * Points within this perpendicular distance from line segment are considered redundant.
 * Set as fraction of accuracy parameter to scale with path resolution.
 */
#define SMARTRTL_SIMPLIFY_EPSILON (_accuracy * 0.5f)

/**
 * Simplification stack buffer size multiplier.
 * The minimum is int((s/2-1)+min(s/2, SMARTRTL_POINTS_MAX-s)), where s = pow(2, floor(log(SMARTRTL_POINTS_MAX)/log(2)))
 * To avoid this annoying math, a good-enough overestimate is ceil(SMARTRTL_POINTS_MAX*2.0f/3.0f)
 * Allocates working memory for recursive simplification algorithm.
 */
#define SMARTRTL_SIMPLIFY_STACK_LEN_MULT (2.0f/3.0f)+1  // simplify buffer size as compared to maximum number of points.
                                                // The minimum is int((s/2-1)+min(s/2, SMARTRTL_POINTS_MAX-s)), where s = pow(2, floor(log(SMARTRTL_POINTS_MAX)/log(2)))
                                                // To avoid this annoying math, a good-enough overestimate is ceil(SMARTRTL_POINTS_MAX*2.0f/3.0f)

/**
 * Maximum execution time for simplification algorithm per iteration in microseconds.
 * Time-slicing prevents blocking the IO thread and impacting flight controller performance.
 */
#define SMARTRTL_SIMPLIFY_TIME_US        200    // maximum time (in microseconds) the simplification algorithm will run before returning

/**
 * Minimum distance in meters between path segments to assume no obstacle between them.
 * Used by loop pruning to determine safe shortcuts. Must be less than _ACCURACY parameter.
 */
#define SMARTRTL_PRUNING_DELTA (_accuracy * 0.99)   // How many meters apart must two points be, such that we can assume that there is no obstacle between them.  must be smaller than _ACCURACY parameter

/**
 * Loop pruning buffer size multiplier relative to maximum points.
 * Allocates working memory for detected loops during pruning algorithm.
 */
#define SMARTRTL_PRUNING_LOOP_BUFFER_LEN_MULT 0.25f // pruning loop buffer size as compared to maximum number of points

/**
 * Maximum execution time for loop detection algorithm per iteration in microseconds.
 * Time-slicing prevents blocking the IO thread and impacting flight controller performance.
 */
#define SMARTRTL_PRUNING_LOOP_TIME_US    200    // maximum time (in microseconds) that the loop finding algorithm will run before returning

/**
 * @class AP_SmartRTL
 * @brief Smart Return-to-Launch path recording and optimization system
 * 
 * @details This class manages the recording, optimization, and retrieval of a vehicle's
 *          flight path to enable intelligent return-to-launch navigation.
 *          
 *          Overall Responsibility:
 *          Records the vehicle's path during flight as a series of waypoints and
 *          generates an optimized return path by removing redundant points and loops.
 *          The system operates continuously in the background, maintaining a memory-
 *          bounded path that can be used for safe return navigation.
 *          
 *          Key Features:
 *          - Automatic path recording at configurable resolution (SMARTRTL_ACCURACY)
 *          - Background cleanup algorithms that run time-sliced in IO thread
 *          - Ramer-Douglas-Peucker simplification to remove collinear points
 *          - Loop detection and pruning for vehicles that circle back on path
 *          - Memory-bounded operation with configurable point limits
 *          - Thread-safe access to path data with semaphore protection
 *          - Automatic deactivation on errors (memory full, GPS loss timeout)
 *          
 *          Integration Points:
 *          - Called from vehicle code at approximately 3Hz via update() method
 *          - Scheduler integration for background cleanup (run_background_cleanup)
 *          - Returns path waypoints via pop_point() during SmartRTL mode
 *          - Coordinates with EKF/AHRS for vehicle position in NED frame
 *          - Integrates with arming system via set_home() for path initialization
 *          
 *          Path Storage:
 *          - Points stored in meters from EKF origin in NED (North-East-Down) frame
 *          - Dynamic memory allocation at init (~20 bytes per point)
 *          - Circular-style access: new points added at end, return reads from end
 *          
 *          Deactivation Conditions:
 *          - Memory allocation failure during initialization
 *          - Path timeout: no points saved for SMARTRTL_TIMEOUT (15 seconds)
 *          - Path full timeout: unable to free space despite cleanup attempts
 *          - Internal error conditions detected
 *          
 * @note This class is called at main loop rate but performs heavy processing in
 *       background thread to avoid impacting flight controller performance.
 * 
 * @warning Once deactivated during a flight, SmartRTL remains disabled until
 *          vehicle is disarmed and rearmed. There is no automatic recovery.
 * 
 * @warning All position/distance parameters use meters from EKF origin in NED frame.
 *          Coordinate frame mismatches will cause navigation errors.
 */
class AP_SmartRTL {

public:

    /**
     * @brief Constructor for AP_SmartRTL
     * 
     * @param[in] example_mode If true, disables logging and background task registration.
     *                         Used only by example sketches for testing. Production code
     *                         should use default value (false).
     * 
     * @note Constructor does not allocate memory. Call init() to allocate path arrays.
     */
    AP_SmartRTL(bool example_mode = false);

    /**
     * @brief Initialize SmartRTL system including memory allocation and background tasks
     * 
     * @details Allocates memory for path storage, simplification stack, and pruning buffers
     *          based on SMARTRTL_POINTS parameter. Registers background cleanup task with
     *          scheduler if not in example mode. Must be called once during vehicle setup
     *          before SmartRTL can be used.
     *          
     *          Memory allocation:
     *          - Path array: _points_max * sizeof(Vector3f) (~12 bytes per point)
     *          - Simplify stack: _points_max * SMARTRTL_SIMPLIFY_STACK_LEN_MULT * sizeof(simplify_start_finish_t)
     *          - Prune loops: _points_max * SMARTRTL_PRUNING_LOOP_BUFFER_LEN_MULT * sizeof(prune_loop_t)
     *          
     *          Total memory: approximately 20-25 bytes per configured point.
     * 
     * @warning If memory allocation fails, SmartRTL will be permanently deactivated.
     *          Failure logged as DEACTIVATED_INIT_FAILED action.
     * 
     * @note Should be called during vehicle setup, not in time-critical sections.
     *       Allocation time depends on configured point count.
     */
    void init();

    /**
     * @brief Check if SmartRTL is currently active and usable
     * 
     * @return true if SmartRTL is active and can record/return path, false if deactivated
     * 
     * @details SmartRTL may become unusable and deactivate for several reasons:
     *          - Memory allocation failure during init()
     *          - Takeoff without valid position estimate
     *          - Path recording timeout (no points saved for SMARTRTL_TIMEOUT ms)
     *          - Path became too full and cleanup algorithms couldn't free space
     *          - Internal error conditions detected
     *          
     *          Once deactivated, SmartRTL remains disabled for remainder of flight.
     *          Must disarm and rearm to reset. Check is_active() before attempting
     *          to use SmartRTL mode.
     * 
     * @note This is a fast check (simple boolean read), safe to call frequently.
     */
    bool is_active() const { return _active; }

    /**
     * @brief Get the current number of points stored in the path
     * 
     * @return Number of points currently in the path array (0 to _path_points_max)
     * 
     * @details Returns the count of valid waypoints in the path. This number changes as:
     *          - Points are added during flight via add_point()
     *          - Points are removed during RTL via pop_point()
     *          - Points are removed by simplification and pruning algorithms
     * 
     * @note Thread-safe: Takes semaphore internally to ensure consistent read.
     *       May return 0 if semaphore cannot be acquired (rare).
     */
    uint16_t get_num_points() const;

    /**
     * @brief Get a specific point from the path by index
     * 
     * @param[in] index Zero-based index into path array (0 to get_num_points()-1)
     * 
     * @return Reference to Vector3f point in meters from EKF origin in NED frame
     * 
     * @details Direct array access for path inspection. Used primarily for visualization
     *          and debugging. Points are stored in chronological order with index 0 being
     *          the oldest (typically home position) and highest index being most recent.
     * 
     * @warning Caller must ensure index < get_num_points(). No bounds checking performed.
     * @warning This does NOT take the semaphore. Only safe when path is not being modified.
     *          Use only for read-only inspection when path updates are paused.
     * 
     * @note Position units: meters from EKF origin in NED (North-East-Down) frame
     */
    const Vector3f& get_point(uint16_t index) const { return _path[index]; }

    /**
     * @brief Add a new waypoint to the end of the path
     * 
     * @param[in] point Position in meters from EKF origin in NED (North-East-Down) frame
     * 
     * @return true if point successfully added, false if failed to acquire semaphore or path full
     * 
     * @details Adds point to path if it's sufficiently far from the last point (distance >
     *          SMARTRTL_ACCURACY parameter). If path is approaching memory limit, triggers
     *          routine cleanup. If path is completely full, logs ADD_FAILED_PATH_FULL and
     *          may deactivate SmartRTL if timeout occurs.
     *          
     *          Distance check: New point must be at least _accuracy meters from last point
     *          to avoid wasting memory on tightly-spaced redundant waypoints.
     *          
     *          Typical call rate: 3Hz from vehicle update loop when save_position is true.
     * 
     * @warning Path coordinates must be in NED frame relative to EKF origin. Incorrect
     *          coordinate frames will cause navigation errors during return.
     * 
     * @warning If path fills up and cannot be cleaned, SmartRTL will deactivate after
     *          SMARTRTL_TIMEOUT (15 seconds) with no successful point additions.
     * 
     * @note Thread-safe: Acquires semaphore before modifying path. Returns false immediately
     *       if semaphore unavailable (logs ADD_FAILED_NO_SEMAPHORE).
     * 
     * @note Updates _last_position_save_ms timestamp on success to reset timeout counter.
     */
    bool add_point(const Vector3f& point);

    /**
     * @brief Get and remove the next waypoint from the return path
     * 
     * @param[out] point Receives the next waypoint position in meters from EKF origin (NED frame)
     * 
     * @return true if point successfully retrieved and removed, false if path empty or semaphore unavailable
     * 
     * @details Retrieves the last point in the path (most recent position) and removes it,
     *          effectively consuming the path backwards toward home. Used during SmartRTL
     *          navigation to get each successive waypoint. Vehicle should navigate to this
     *          point, then call pop_point() again for the next waypoint.
     *          
     *          Path consumption: Points are removed from end of array (highest index), so
     *          vehicle follows recorded path in reverse chronological order back to home.
     *          
     *          Failure cases:
     *          - Path is empty (all points consumed): Returns false, point unchanged
     *          - Cannot acquire semaphore: Returns false, logs POP_FAILED_NO_SEMAPHORE
     * 
     * @warning Point coordinates are in NED frame relative to EKF origin. Navigation code
     *          must use same coordinate frame to avoid position errors.
     * 
     * @note Thread-safe: Attempts to acquire semaphore before accessing path. Returns
     *       immediately if semaphore held by background cleanup thread.
     * 
     * @note Updates _path_points_completed_limit to inform simplify/prune algorithms
     *       that path is being consumed and they should not process removed points.
     */
    bool pop_point(Vector3f& point);

    /**
     * @brief Look at the next waypoint without removing it from the path
     * 
     * @param[out] point Receives the next waypoint position in meters from EKF origin (NED frame)
     * 
     * @return true if point successfully retrieved, false if path empty or semaphore unavailable
     * 
     * @details Retrieves the last point in the path without removing it, allowing preview
     *          of next navigation target. Useful for distance/bearing calculations or
     *          deciding whether to continue SmartRTL. Path remains unchanged after this call.
     *          
     *          Typical use: Check distance to next point before committing to navigation,
     *          or verify path sanity during SmartRTL initialization.
     *          
     *          Failure cases:
     *          - Path is empty: Returns false, point unchanged
     *          - Cannot acquire semaphore: Returns false, logs PEEK_FAILED_NO_SEMAPHORE
     * 
     * @note Thread-safe but non-blocking: Returns immediately if semaphore held by IO
     *       thread. This may fail temporarily during background cleanup operations.
     * 
     * @note Position units: meters from EKF origin in NED (North-East-Down) frame
     */
    bool peek_point(Vector3f& point);

    /**
     * @brief Clear path and set home position for new flight
     * 
     * @param[in] position_ok True if vehicle has valid position estimate from EKF/AHRS
     * 
     * @details Resets SmartRTL for a new flight by clearing the path and recording the
     *          current position as home. Should be called as part of arming procedure.
     *          
     *          If position_ok is false:
     *          - SmartRTL will be deactivated (takeoff without GPS lock)
     *          - Path is cleared but no home point is saved
     *          - Vehicle cannot use SmartRTL for this flight
     *          
     *          If position_ok is true:
     *          - Current position (from EKF/AHRS) saved as first path point
     *          - SmartRTL activated and ready to record path
     *          - Timeout counters reset
     * 
     * @warning Must be called when vehicle has valid EKF origin. Calling before EKF
     *          initialization may result in incorrect home position.
     * 
     * @note This overload retrieves position from EKF/AHRS internally. Example sketches
     *       should use the overload that accepts current_pos parameter directly.
     * 
     * @see set_home(bool, const Vector3f&) for version with explicit position
     */
    void set_home(bool position_ok);

    /**
     * @brief Clear path and set home position using provided location
     * 
     * @param[in] position_ok True if vehicle has valid position estimate
     * @param[in] current_pos Current vehicle position in meters from EKF origin (NED frame)
     * 
     * @details Variant of set_home() that accepts position explicitly rather than querying
     *          EKF/AHRS. Used by example sketches where position is maintained externally.
     *          Behavior otherwise identical to set_home(bool).
     * 
     * @warning current_pos must be in NED frame relative to EKF origin. Incorrect frame
     *          will cause all subsequent path points to be misaligned.
     * 
     * @note This is the implementation used by example sketches and test code where EKF
     *       may not be available or position is simulated.
     */
    void set_home(bool position_ok, const Vector3f& current_pos);

    /**
     * @brief Update SmartRTL state and conditionally save current position to path
     * 
     * @param[in] position_ok True if vehicle has valid position estimate from EKF/AHRS
     * @param[in] save_position True if current position should be saved to path
     * 
     * @details Main update function called regularly from vehicle code. Performs several tasks:
     *          - Updates position validity timestamp if position_ok is true
     *          - Adds current position to path if save_position is true
     *          - Checks for timeout conditions (no valid position or no saved positions)
     *          - Deactivates SmartRTL if SMARTRTL_TIMEOUT (15 seconds) exceeded
     *          
     *          Recommended call rate: 3Hz or higher for adequate path resolution.
     *          Lower rates will result in larger gaps between waypoints.
     *          
     *          Position saving logic:
     *          - If save_position is false: Only updates timeout tracking, no path changes
     *          - If save_position is true: Calls add_point() with current position from EKF
     *          
     *          Timeout deactivation:
     *          - If position_ok false for >15 seconds: Deactivates with DEACTIVATED_BAD_POSITION_TIMEOUT
     *          - If no points saved for >15 seconds: Deactivates with DEACTIVATED_PATH_FULL_TIMEOUT
     * 
     * @warning Must be called regularly even when not in SmartRTL mode to maintain path
     *          recording. Vehicle code should call this continuously during flight.
     * 
     * @note This overload queries current position from EKF/AHRS internally. Example
     *       sketches should use update(bool, const Vector3f&) with explicit position.
     * 
     * @note Call frequency affects path resolution: 3Hz = ~2m spacing at 6m/s flight speed
     *       (combined with SMARTRTL_ACCURACY parameter)
     */
    void update(bool position_ok, bool save_position);

    /**
     * @brief Update SmartRTL using explicitly provided position
     * 
     * @param[in] position_ok True if provided position is valid
     * @param[in] current_pos Current vehicle position in meters from EKF origin (NED frame)
     * 
     * @details Variant of update() that accepts position explicitly rather than querying
     *          EKF/AHRS. Used by example sketches. Always saves position if position_ok
     *          is true (equivalent to calling update(position_ok, true) in standard version).
     * 
     * @warning current_pos must be in NED frame relative to EKF origin. Incorrect frame
     *          will cause navigation errors during SmartRTL.
     * 
     * @note Example sketch version - production vehicle code should use standard update()
     */
    void update(bool position_ok, const Vector3f& current_pos);

    /**
     * @enum ThoroughCleanupType
     * @brief Cleanup operation modes for thorough path optimization
     * 
     * @details Specifies which cleanup algorithms to run during thorough cleanup.
     *          Normal vehicle code uses THOROUGH_CLEAN_DEFAULT. Other modes exist
     *          for testing and debugging specific algorithms in example sketches.
     */
    enum ThoroughCleanupType {
        THOROUGH_CLEAN_DEFAULT = 0,     ///< Perform both simplify and prune (production mode for vehicle code)
        THOROUGH_CLEAN_ALL,             ///< Same as DEFAULT, used by example sketches for clarity
        THOROUGH_CLEAN_SIMPLIFY_ONLY,   ///< Perform only Ramer-Douglas-Peucker simplification (example sketch testing)
        THOROUGH_CLEAN_PRUNE_ONLY,      ///< Perform only loop pruning (example sketch testing)
    };

    /**
     * @brief Request comprehensive path optimization before SmartRTL navigation
     * 
     * @param[in] clean_type Type of cleanup to perform (default: both simplify and prune)
     * 
     * @return true if cleanup completed, false if still in progress
     * 
     * @details Triggers an intensive path optimization that runs to completion before
     *          returning control. Unlike routine cleanup (which processes incrementally),
     *          thorough cleanup processes the entire path:
     *          
     *          Simplification (Ramer-Douglas-Peucker):
     *          - Removes all points that lie within SMARTRTL_SIMPLIFY_EPSILON of straight line segments
     *          - Processes entire path from start to finish
     *          - Typically removes 30-50% of points on relatively straight paths
     *          
     *          Loop Pruning:
     *          - Detects all loops where path crosses back over itself
     *          - Removes interior loop points, cutting directly to exit point
     *          - Can dramatically shorten path if vehicle circled or searched area
     *          
     *          Execution model:
     *          - Call repeatedly until it returns true (cleanup complete)
     *          - Each call does partial work (time-sliced to avoid blocking)
     *          - Background thread continues processing between calls
     *          - Typical completion time: 100-500ms depending on path complexity
     *          
     *          Usage pattern:
     *          When SmartRTL mode is entered, call this function at ~10Hz until
     *          it returns true, then begin pop_point() navigation.
     * 
     * @warning This is blocking in that navigation should NOT begin until cleanup
     *          completes. Vehicle should loiter during this time (~0.5s typically).
     * 
     * @note clean_type parameter is for example sketches only. Vehicle code should
     *       use default value to perform full cleanup.
     * 
     * @note Completion time varies with path length and complexity. Simple paths
     *       complete in 1-2 iterations, complex paths may take 10+ iterations.
     */
    bool request_thorough_cleanup(ThoroughCleanupType clean_type = THOROUGH_CLEAN_DEFAULT);

    /**
     * @brief Cancel an in-progress thorough cleanup request
     * 
     * @details Clears the thorough cleanup request flag, stopping the background thread
     *          from continuing the intensive cleanup. Useful if SmartRTL mode is exited
     *          before cleanup completes, allowing background thread to return to routine
     *          maintenance mode.
     *          
     *          Typical usage: Vehicle exits SmartRTL mode before cleanup finishes (e.g.,
     *          pilot takes manual control). Call this to avoid wasting CPU on unnecessary
     *          comprehensive optimization.
     * 
     * @note Safe to call even if no thorough cleanup is in progress.
     * @note Does not undo cleanup already performed, only stops future work.
     */
    void cancel_request_for_thorough_cleanup();

    /**
     * @brief Execute time-sliced cleanup algorithms in background thread
     * 
     * @details Core background processing function that should be called regularly from
     *          the IO/scheduler thread. Performs incremental path optimization:
     *          
     *          Routine mode (when thorough cleanup not requested):
     *          - Runs simplification incrementally on new path points
     *          - Runs loop detection incrementally on simplified points
     *          - Removes points when cleanup benefit exceeds SMARTRTL_CLEANUP_POINT_MIN
     *          - Each call limited to SMARTRTL_SIMPLIFY_TIME_US or SMARTRTL_PRUNING_LOOP_TIME_US
     *          
     *          Thorough cleanup mode (when requested):
     *          - Focuses on completing full path simplification first
     *          - Then completes full loop pruning
     *          - Updates _thorough_clean_complete_ms when finished
     *          
     *          Time-slicing behavior:
     *          - Simplify: Max 200us per call (SMARTRTL_SIMPLIFY_TIME_US)
     *          - Prune: Max 200us per call (SMARTRTL_PRUNING_LOOP_TIME_US)
     *          - Prevents blocking IO thread and impacting flight controller performance
     * 
     * @warning Must be registered with scheduler and called from IO thread.
     *          Do NOT call from main thread or other time-critical contexts.
     * 
     * @note Registered automatically during init() unless in example_mode.
     * @note Execution time per call: <200us guaranteed by time-slicing.
     * @note Called at scheduler rate (typically 10-50Hz depending on IO thread loading).
     */
    void run_background_cleanup();

    /**
     * @brief Check if pilot yaw input should control heading during SmartRTL
     * 
     * @return true if pilot yaw should be used, false if vehicle should face along path
     * 
     * @details Determines heading control mode during SmartRTL navigation. Returns false
     *          if SRTL_OPTIONS bit 2 (IgnorePilotYaw) is set, allowing vehicle to face
     *          along path direction. Returns true otherwise, allowing pilot to control
     *          yaw independently of path following.
     *          
     *          Integration with vehicle modes:
     *          - Vehicle SmartRTL mode calls this to decide yaw control authority
     *          - If true: Pilot yaw stick controls heading, vehicle may fly sideways to path
     *          - If false: Vehicle automatically faces tangent to path, pilot yaw ignored
     * 
     * @note This is a configuration check (parameter bit test), not dynamic state.
     *       Return value is consistent throughout flight unless parameters change.
     */
    bool use_pilot_yaw(void) const;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @enum Action
     * @brief Event types for SmartRTL logging and diagnostics
     * 
     * @details Identifies specific events and errors for logging to dataflash and
     *          ground station notifications. Used by log_action() to record SmartRTL
     *          state changes and operational events for post-flight analysis.
     */
    enum Action : uint8_t {
        POINT_ADD = 0,                          ///< Normal: Point successfully added to path
        POINT_PRUNE = 1,                        ///< Normal: Points removed by loop pruning algorithm
        POINT_SIMPLIFY = 2,                     ///< Normal: Points removed by simplification algorithm
        ADD_FAILED_NO_SEMAPHORE = 3,            ///< Warning: add_point() couldn't acquire semaphore (IO thread busy)
        ADD_FAILED_PATH_FULL = 4,               ///< Warning: add_point() failed because path array is full
        POP_FAILED_NO_SEMAPHORE = 5,            ///< Warning: pop_point() couldn't acquire semaphore (IO thread busy)
        PEEK_FAILED_NO_SEMAPHORE = 6,           ///< Warning: peek_point() couldn't acquire semaphore (IO thread busy)
        DEACTIVATED_INIT_FAILED = 7,            ///< Critical: init() memory allocation failed, SmartRTL disabled
        // DEACTIVATED_BAD_POSITION = 8,        ///< Unused (historical), value reserved
        DEACTIVATED_BAD_POSITION_TIMEOUT = 9,   ///< Critical: No valid position for >15s, SmartRTL disabled
        DEACTIVATED_PATH_FULL_TIMEOUT = 10,     ///< Critical: Path full for >15s, SmartRTL disabled
        DEACTIVATED_PROGRAM_ERROR = 11,         ///< Critical: Internal error detected, SmartRTL disabled
    };

    /**
     * @enum Options
     * @brief Configuration bitmask for SRTL_OPTIONS parameter
     * 
     * @details Bit flags that modify SmartRTL behavior. Set via SRTL_OPTIONS parameter.
     *          Multiple options can be combined with bitwise OR.
     */
    enum class Options : int32_t {
        // bits 0 and 1 are available for future use
        IgnorePilotYaw    = (1U << 2),  ///< Bit 2: If set, vehicle faces along path; if clear, pilot controls yaw
    };

    // routine cleanup attempts to remove 10 points (see SMARTRTL_CLEANUP_POINT_MIN definition) by simplification or loop pruning
    void routine_cleanup(uint16_t path_points_count, uint16_t path_points_complete_limit);

    // thorough cleanup simplifies and prunes all loops.  returns true if the cleanup was completed.
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    bool thorough_cleanup(uint16_t path_points_count, ThoroughCleanupType clean_type);

    // the two cleanup steps run from the background thread
    // these are public so that they can be tested by the example sketch
    void detect_simplifications();
    void detect_loops();

    // restart simplify or pruning if new points have been added to path
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    void restart_simplify_if_new_points(uint16_t path_points_count);

    // restart pruning if new points have been simplified
    void restart_pruning_if_new_points();

    // restart simplify algorithm so that detect_simplify will check all new points that have been added
    // to the path since it last completed.
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    void restart_simplification(uint16_t path_points_count);

    // reset simplify algorithm so that it will re-check all points in the path
    void reset_simplification();

    // restart pruning algorithm so that detect_loops will check all new points that have been added
    // to the path since it last completed.
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    void restart_pruning(uint16_t path_points_count);

    // reset pruning algorithm so that it will re-check all points in the path
    void reset_pruning();

    // remove all simplify-able points from the path
    void remove_points_by_simplify_bitmask();

    // remove loops until at least num_point_to_remove have been removed from path
    // does not necessarily prune all loops
    // returns false if it failed to remove points (because it could not take semaphore)
    bool remove_points_by_loops(uint16_t num_points_to_remove);

    // add loop to loops array
    //  returns true if loop added successfully, false on failure (because loop array is full)
    //  checks if loop overlaps with an existing loop, keeps only the longer loop
    //  example: segment_a(point2~point3) overlaps with segment_b (point5~point6), add_loop(3,5,midpoint)
    bool add_loop(uint16_t start_index, uint16_t end_index, const Vector3f& midpoint);

    /**
     * @struct dist_point
     * @brief Result structure for segment-to-segment distance calculations
     * 
     * @details Holds the result of calculating the closest approach between two line
     *          segments in 3D space. Used by loop detection algorithm to find where
     *          the flight path comes close to crossing itself.
     */
    typedef struct {
        float distance;         ///< Closest distance between the two line segments in meters
        Vector3f midpoint;      ///< Point exactly halfway between the two closest points in meters (NED frame from EKF origin)
    } dist_point;

    // get the closest distance between 2 line segments and the point midway between the closest points
    static dist_point segment_segment_dist(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, const Vector3f& p4);

    // de-activate SmartRTL, send warning to GCS and logger
    void deactivate(Action action, const char *reason);

#if HAL_LOGGING_ENABLED
    // logging
    void log_action(Action action, const Vector3f &point = Vector3f()) const;
#else
    void log_action(Action action, const Vector3f &point = Vector3f()) const {}
#endif

    // parameters
    AP_Float _accuracy;     ///< SMARTRTL_ACCURACY: Minimum distance between path points in meters
    AP_Int16 _points_max;   ///< SMARTRTL_POINTS: Maximum number of path points to store (memory allocation size)
    AP_Int32 _options;      ///< SRTL_OPTIONS: Bitmask of configuration options (see Options enum)

    // SmartRTL State Variables
    bool _active;       ///< True if SmartRTL is usable; false if deactivated due to errors or timeout
    bool _example_mode; ///< True when running in example sketch mode (disables logging and scheduler registration)
    bool _home_saved;   ///< True once home position has been successfully saved by set_home() or update()
    uint32_t _last_good_position_ms;    ///< Timestamp (milliseconds) when last valid position was received; used for position timeout detection
    uint32_t _last_position_save_ms;    ///< Timestamp (milliseconds) when last point was saved to path; used for save timeout detection  
    uint32_t _thorough_clean_request_ms;///< Timestamp (milliseconds) when thorough cleanup was requested; signals background thread to start intensive cleanup
    uint32_t _thorough_clean_complete_ms; ///< Set to _thorough_clean_request_ms when background thread completes thorough cleanup; indicates completion
    uint32_t _last_low_space_notify_ms; ///< Timestamp (milliseconds) of last low-space MAVLink warning; used to throttle notifications
    ThoroughCleanupType _thorough_clean_type;   ///< Type of thorough cleanup to perform (used by example sketches to test algorithms individually)

    /**
     * @brief Path storage array - dynamically allocated at init()
     * @details Points stored in meters from EKF origin in NED (North-East-Down) frame.
     *          Array size is _path_points_max (typically 300-500 points).
     *          Access protected by _path_sem semaphore for thread-safety.
     * @warning All position coordinates are NED frame relative to EKF origin.
     */
    Vector3f* _path;    // points are stored in meters from EKF origin in NED
    uint16_t _path_points_max;  ///< Actual allocated array size; fixed at init(), independent of parameter changes during flight
    uint16_t _path_points_count;///< Current number of valid points in path array (0 to _path_points_max)
    uint16_t _path_points_completed_limit;  ///< Marks how many points have been consumed by pop_point(); cleanup algorithms ignore points beyond this
    HAL_Semaphore _path_sem;   ///< Semaphore protecting _path array access between main thread and IO thread

    /**
     * @brief Ramer-Douglas-Peucker simplification algorithm state
     * @details Maintains working state for incremental path simplification that runs
     *          time-sliced in background thread. Simplification removes points that
     *          lie within epsilon distance of straight line segments connecting
     *          surrounding points.
     */
    
    /**
     * @struct simplify_start_finish_t
     * @brief Stack element for recursive simplification algorithm
     * @details Defines a path segment [start, finish] that needs simplification checking.
     *          Stack holds pending segments to process in iterative implementation of
     *          recursive Ramer-Douglas-Peucker algorithm.
     */
    typedef struct {
        uint16_t start;     ///< Starting index of path segment to simplify
        uint16_t finish;    ///< Ending index of path segment to simplify
    } simplify_start_finish_t;
    
    /**
     * @brief Simplification algorithm working state and buffers
     */
    struct {
        bool complete;          ///< True when simplification has processed all points up to path_points_completed
        bool removal_required;  ///< True if simplify-able points found; triggers actual removal step
        uint16_t path_points_count; ///< Snapshot of _path_points_count when algorithm started; detects if path changed
        uint16_t path_points_completed = SMARTRTL_POINTS_MAX; ///< Points with index < this have been simplified; algorithm processes from here
        simplify_start_finish_t* stack; ///< Working stack for iterative Ramer-Douglas-Peucker algorithm (dynamically allocated)
        uint16_t stack_max;     ///< Maximum stack depth (allocated size = _points_max * SMARTRTL_SIMPLIFY_STACK_LEN_MULT)
        uint16_t stack_count;   ///< Current number of segments on stack waiting for processing
        Bitmask<SMARTRTL_POINTS_MAX> bitmask;  ///< Bit cleared for each removable point; bit set means point must be kept
    } _simplify;

    /**
     * @brief Loop detection and pruning algorithm state
     * @details Maintains working state for incremental loop detection that runs
     *          time-sliced in background thread. Loop pruning detects where the
     *          path crosses back over itself and removes the interior loop points,
     *          creating a shortcut directly to where path exits the loop.
     */
    
    /**
     * @struct prune_loop_t
     * @brief Detected loop information for path pruning
     * @details Represents a detected loop in the path where vehicle circled back
     *          on itself. Contains indices of loop start/end and the shortcut point
     *          that should replace the loop when removed.
     */
    typedef struct {
        uint16_t start_index;   ///< Index of first point in loop (where loop begins)
        uint16_t end_index;     ///< Index of last point in loop (where path exits loop)
        Vector3f midpoint;      ///< Shortcut point in meters (NED frame from EKF origin) that replaces start_index when loop removed
        float length_squared;   ///< Loop length squared in meters²; used to prioritize removing longest loops first
    } prune_loop_t;
    
    /**
     * @brief Loop pruning algorithm working state and buffers
     */
    struct {
        bool complete;          ///< True when loop detection has checked all point pairs up to path_points_completed
        uint16_t path_points_count;  ///< Snapshot of _path_points_count when algorithm started; detects if path changed
        uint16_t path_points_completed; ///< Points with index < this have been checked for loops; algorithm processes from here
        uint16_t i;     ///< Outer loop index for pairwise segment comparison (first segment)
        uint16_t j;     ///< Inner loop index for pairwise segment comparison (second segment)
        prune_loop_t* loops;///< Array of detected loops (dynamically allocated, sorted by length_squared descending)
        uint16_t loops_max; ///< Maximum loops that can be stored (allocated size = _points_max * SMARTRTL_PRUNING_LOOP_BUFFER_LEN_MULT)
        uint16_t loops_count;   ///< Current number of detected loops in array waiting for removal
    } _prune;

    // returns true if the two loops overlap (used within add_loop to determine which loops to keep or throw away)
    bool loops_overlap(const prune_loop_t& loop1, const prune_loop_t& loop2) const;
};
