#pragma once

/**
 * @file AP_GPS_FixType.h
 * @brief GPS fix type enumeration used across ArduPilot
 * 
 * @details Defines standard GPS fix quality levels from NO_GPS to RTK_FIXED.
 *          This is a library-independent header that can be used even when
 *          AP_GPS is not compiled in, allowing other modules to reference
 *          GPS fix types without creating circular dependencies.
 * 
 * @note Not enum class to allow comparison operators for "fix at least this good"
 *       checks using < and > operators throughout the codebase.
 */

/**
 * @enum AP_GPS_FixType
 * @brief GPS fix quality enumeration in increasing order of accuracy
 * 
 * @details Fix types ordered from worst (NO_GPS=0) to best (RTK_FIXED=6) to
 *          enable comparison operations. Each fix type indicates progressively
 *          better position accuracy:
 *          - NO_GPS/NONE: No usable position (accuracy undefined, do not use for navigation)
 *          - FIX_2D: Horizontal position only (accuracy typically >10m, insufficient altitude)
 *          - FIX_3D: Full 3D position (accuracy typically 5-15m)
 *          - DGPS: Differential corrections applied (accuracy typically 1-5m)
 *          - RTK_FLOAT: RTK ambiguities not resolved (accuracy typically 0.1-1m)
 *          - RTK_FIXED: RTK ambiguities resolved (accuracy typically 1-5cm)
 * 
 * @note Comparable using < and > operators for minimum fix quality checks
 * @warning EKF initialization typically requires >= FIX_3D before arming
 * 
 * Usage examples:
 * @code
 * // Check for minimum 3D fix
 * if (gps.status() >= AP_GPS_FixType::FIX_3D) {
 *     // Safe to use position for navigation
 * }
 * 
 * // Check for RTK solution
 * if (gps.status() >= AP_GPS_FixType::RTK_FLOAT) {
 *     // High-precision positioning available
 * }
 * @endcode
 */

enum class AP_GPS_FixType {
    /**
     * No GPS hardware detected or GPS communication failed.
     * State indicates no GPS receiver present on configured port.
     * @warning Vehicle will not arm with NO_GPS on any configured GPS instance
     */
    NO_GPS = 0,
    
    /**
     * GPS receiver detected and communicating but no position lock.
     * Receiving satellite signals but insufficient for position solution.
     * Typically occurs during GPS warm-up (30-60s) or poor sky visibility.
     * @note Some systems report this during initialization with valid time but no position
     */
    NONE = 1,
    
    /**
     * 2-dimensional position fix (latitude/longitude only).
     * Insufficient satellites/geometry for altitude solution.
     * Horizontal accuracy typically >10m.
     * @warning Insufficient for flying vehicles, may be acceptable for rovers in flat terrain
     */
    FIX_2D = 2,
    
    /**
     * 3-dimensional position fix (latitude/longitude/altitude).
     * Standard autonomous GPS accuracy (typically 5-15m horizontal, 10-25m vertical).
     * Minimum fix type for ArduPilot flight operations.
     * @note Most ArduPilot vehicles require >= FIX_3D for EKF initialization and arming
     */
    FIX_3D = 3,
    
    /**
     * Differential GPS corrections applied (SBAS/WAAS/EGNOS).
     * Improved accuracy over autonomous (typically 1-5m horizontal, 2-8m vertical).
     * Corrections from ground-based reference stations or SBAS satellites.
     * @note DGPS usually provides sufficient accuracy for precision agriculture and surveying
     */
    DGPS = 4,
    
    /**
     * Real-Time Kinematic with floating-point carrier-phase ambiguities.
     * High accuracy but ambiguities not yet converged to integers.
     * Typical accuracy 0.1-1m (10-100cm).
     * @note RTK Float often transitions to RTK Fixed within 30s-5min depending on baseline and conditions
     */
    RTK_FLOAT = 5,
    
    /**
     * Real-Time Kinematic with integer carrier-phase ambiguity resolution.
     * Highest accuracy GPS solution (typically 1-5cm horizontal, 2-10cm vertical).
     * Requires baseline to RTK base station (usually <20km).
     * @note RTK Fixed accuracy enables centimeter-level precision landing and surveying
     */
    RTK_FIXED = 6,
};
