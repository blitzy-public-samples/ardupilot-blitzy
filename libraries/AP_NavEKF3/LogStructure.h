/**
 * @file LogStructure.h
 * @brief EKF3 binary logging structures and registration system
 * 
 * @details This file defines the binary log message structures used by NavEKF3 (Navigation Extended Kalman Filter 3)
 *          for high-speed data recording during flight. These structures are designed for:
 *          - Compact binary representation to minimize storage and bandwidth requirements
 *          - High-frequency logging (typically 10-25 Hz) of estimator states and diagnostics
 *          - Post-flight analysis and algorithm debugging
 *          - Ground station real-time telemetry display
 * 
 *          The log structures use packed binary format with scaled integer fields where possible
 *          to reduce message sizes. For example, angles are stored as int16_t in centidegrees rather
 *          than float in radians, achieving 4:1 size reduction while maintaining sufficient precision.
 * 
 *          Log Message Categories:
 *          - XKF0: Beacon positioning sensor diagnostics
 *          - XKF1: Primary estimator outputs (attitude, velocity, position, gyro bias)
 *          - XKF2: Secondary outputs (accel bias, wind, magnetometer, drag/sideslip)
 *          - XKF3: Innovation values for sensor measurements
 *          - XKF4: Variance estimates and filter status
 *          - XKF5: Optical flow and rangefinder innovations, error magnitudes
 *          - XKFD: Body frame odometry errors
 *          - XKFM: On-ground-not-moving diagnostic data
 *          - XKFS: Sensor selection indices
 *          - XKQ: Attitude quaternion representation
 *          - XKT: Timing and sample rate diagnostics
 *          - XKTV: Yaw estimator tilt error variances
 *          - XKV1/XKV2: Complete EKF state variance vector (split across two messages)
 * 
 *          Each log structure includes:
 *          - LOG_PACKET_HEADER: Standard header with message type and length
 *          - time_us: Microsecond timestamp for precise time correlation
 *          - core: EKF core instance identifier (0-2 for multi-EKF configurations)
 *          - Scaled sensor/state data optimized for size and precision
 * 
 * @note All structures use PACKED attribute to eliminate padding bytes and ensure consistent
 *       binary layout across different compilers and architectures.
 * 
 * @warning These structures define the on-disk and over-the-wire binary format. Changes to field
 *          types, ordering, or sizes will break compatibility with existing ground station software,
 *          log analysis tools, and historical log files. When modifications are necessary:
 *          - Create new message types with incremented suffixes (e.g., XKF6)
 *          - Maintain old message types for backward compatibility
 *          - Update ground station message definitions in sync
 *          - Document format changes in release notes
 * 
 * @see AP_Logger for the logging infrastructure and message registration system
 * @see NavEKF3_core.h for the EKF algorithm implementation that generates these log values
 * @see LogStructure.cpp in AP_Logger for format string documentation
 * 
 * Source: libraries/AP_NavEKF3/LogStructure.h:1-463
 */

#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_AHRS/AP_AHRS_config.h>

/**
 * @brief EKF3 log message ID enumeration for registration with AP_Logger
 * 
 * @details This macro expands to a comma-separated list of log message IDs that are registered
 *          with the AP_Logger system during initialization. Each ID corresponds to a specific
 *          log message type (XKF0, XKF1, etc.) and is used by the logging system to:
 *          - Allocate message type numbers in the global log message registry
 *          - Enable/disable specific message types via logging parameters
 *          - Route log data to appropriate storage backends (SD card, flash, MAVLink)
 * 
 *          This macro is typically used in the LOG_MSG enumeration in AP_Logger/LogStructure.h
 *          to incorporate EKF3 messages into the system-wide message catalog.
 * 
 * @note The order of IDs in this list does not affect functionality but should remain consistent
 *       for code maintainability. Message IDs are assigned dynamically at compile time.
 * 
 * @see LOG_STRUCTURE_FROM_NAVEKF3 for the corresponding structure definitions
 * @see AP_Logger/LogStructure.h for the global message registration system
 */
#define LOG_IDS_FROM_NAVEKF3 \
    LOG_XKF0_MSG, \
    LOG_XKF1_MSG, \
    LOG_XKF2_MSG, \
    LOG_XKF3_MSG, \
    LOG_XKF4_MSG, \
    LOG_XKF5_MSG, \
    LOG_XKFD_MSG, \
    LOG_XKFM_MSG, \
    LOG_XKFS_MSG, \
    LOG_XKQ_MSG,  \
    LOG_XKT_MSG,  \
    LOG_XKTV_MSG, \
    LOG_XKV1_MSG, \
    LOG_XKV2_MSG, \
    LOG_XKY0_MSG, \
    LOG_XKY1_MSG

// @LoggerMessage: XKF0
// @Description: EKF3 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: ID: Beacon sensor ID
// @Field: rng: Beacon range
// @Field: innov: Beacon range innovation
// @Field: SIV: sqrt of beacon range innovation variance
// @Field: TR: Beacon range innovation consistency test ratio
// @Field: BPN: Beacon north position
// @Field: BPE: Beacon east position
// @Field: BPD: Beacon down position
// @Field: OFH: High estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFL: Low estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFN: North position of receiver rel to EKF origin
// @Field: OFE: East position of receiver rel to EKF origin
// @Field: OFD: Down position of receiver rel to EKF origin

/**
 * @brief Binary log structure for EKF3 beacon positioning system diagnostics
 * 
 * @details Records innovation (measurement residual) data for beacon-based positioning systems.
 *          Beacons provide range measurements to known fixed positions, enabling indoor or GPS-denied
 *          navigation. This message logs the difference between predicted and measured beacon ranges,
 *          along with statistical measures (variance, test ratio) used to accept/reject measurements.
 *          
 *          All position fields use centimeter scaling (int16_t) for compact storage, providing
 *          ±327 meter range with 1cm resolution. Range innovations help diagnose beacon configuration
 *          issues, multipath interference, or EKF state estimation errors.
 * 
 * @note Logged only when beacon positioning is active and configured (AP_Beacon library enabled).
 *       Typical logging rate: 10 Hz per active beacon.
 */
struct PACKED log_XKF0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t ID;             // beacon identifier
    int16_t rng;            // beacon range (cm)
    int16_t innov;          // beacon range innovation (cm)
    uint16_t sqrtInnovVar;  // sqrt of beacon range innovation variance (cm)
    uint16_t testRatio;     // beacon range innovation consistency test ratio *100
    int16_t beaconPosN;     // beacon north position (cm)
    int16_t beaconPosE;     // beacon east position (cm)
    int16_t beaconPosD;     // beacon down position (cm)
    int16_t offsetHigh;     // high estimate of vertical position offset of beacons rel to EKF origin (cm)
    int16_t offsetLow;      // low estimate of vertical position offset of beacons rel to EKF origin (cm)
    int16_t posN;           // North position of receiver rel to EKF origin (cm)
    int16_t posE;           // East position of receiver rel to EKF origin (cm)
    int16_t posD;           // Down position of receiver rel to EKF origin (cm)
};


// @LoggerMessage: XKF1
// @Description: EKF3 estimator outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: VN: Estimated velocity (North component)
// @Field: VE: Estimated velocity (East component)
// @Field: VD: Estimated velocity (Down component)
// @Field: dPD: Filtered derivative of vertical position (down)
// @Field: PN: Estimated distance from origin (North component)
// @Field: PE: Estimated distance from origin (East component)
// @Field: PD: Estimated distance from origin (Down component)
// @Field: GX: Estimated gyro bias, X axis
// @Field: GY: Estimated gyro bias, Y axis
// @Field: GZ: Estimated gyro bias, Z axis
// @Field: OH: Height of origin above WGS-84

/**
 * @brief Primary EKF3 state estimates - attitude, velocity, position, and gyro bias
 * 
 * @details This is the most important EKF3 log message, containing the core navigation solution
 *          used for flight control. It includes:
 *          - Attitude: Roll/pitch in centidegrees (±180°), yaw in centidegrees (0-360°)
 *          - Velocity: NED frame in m/s (float precision for accuracy)
 *          - Position: NED frame relative to EKF origin in meters (float)
 *          - Gyro bias: Estimated sensor errors in body frame (centidegrees/sec)
 *          - Origin height: EKF reference altitude above WGS-84 ellipsoid
 * 
 *          The EKF origin is established at initialization (typically first GPS lock) and remains
 *          fixed unless explicitly reset. All position values are relative to this origin.
 *          
 *          Attitude uses scaled integers (int16_t centidegrees) for roll/pitch to save space,
 *          while velocities and positions use float for the precision required by navigation.
 *          The posD_dot field provides filtered vertical velocity for altitude hold controllers.
 * 
 * @note This message is logged at high rate (typically 10-25 Hz) as it represents the primary
 *       navigation solution. Used extensively in post-flight analysis and real-time telemetry.
 * 
 * @warning Ground control stations depend on this message format for attitude/position display.
 *          Field order and types must remain stable across firmware versions.
 */
struct PACKED log_XKF1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posD_dot;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
    int32_t originHgt;
};


// @LoggerMessage: XKF2
// @Description: EKF3 estimator secondary outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: AX: Estimated accelerometer X bias
// @Field: AY: Estimated accelerometer Y bias
// @Field: AZ: Estimated accelerometer Z bias
// @Field: VWN: Estimated wind velocity (moving-to-North component)
// @Field: VWE: Estimated wind velocity (moving-to-East component)
// @Field: MN: Magnetic field strength (North component)
// @Field: ME: Magnetic field strength (East component)
// @Field: MD: Magnetic field strength (Down component)
// @Field: MX: Magnetic field strength (body X-axis)
// @Field: MY: Magnetic field strength (body Y-axis)
// @Field: MZ: Magnetic field strength (body Z-axis)
// @Field: IDX: Innovation in vehicle drag acceleration (X-axis component)
// @Field: IDY: Innovation in vehicle drag acceleration (Y-axis component)
// @Field: IS: Innovation in vehicle sideslip

/**
 * @brief Secondary EKF3 state estimates - sensor biases, wind, magnetometer, and aerodynamics
 * 
 * @details Complementary to XKF1, this message logs additional estimated states:
 *          - Accelerometer bias: Body-frame sensor errors in cm/s² (scaled int16_t)
 *          - Wind velocity: Earth-frame (NED) wind estimation in cm/s for fixed-wing energy management
 *          - Magnetometer: Earth-frame (NED) and body-frame field strengths in milligauss
 *          - Drag innovations: Aerodynamic model residuals in m/s² (float) for fixed-wing
 *          - Sideslip innovation: Lateral aerodynamic model residual in radians (float)
 * 
 *          Magnetometer fields are stored in both NED (for declination/inclination) and body frame
 *          (for magnetometer calibration validation). Drag and sideslip innovations are used only
 *          for fixed-wing aircraft with aerodynamic fusion enabled (EK3_DRAG_CTRL parameter).
 * 
 *          Scaling: Accelerometer and wind use cm/s (int16_t) for ±327 m/s range with cm resolution.
 *          Magnetometer uses milligauss (int16_t) for Earth field strengths typically 200-600 mG.
 * 
 * @note Wind estimation requires airspeed sensor or GPS velocity comparison during maneuvering.
 *       Magnetometer fusion can be disabled (EK3_MAG_CAL parameter) for indoor/GPS-based navigation.
 */
struct PACKED log_XKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t accBiasX;
    int16_t accBiasY;
    int16_t accBiasZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    float innovDragX;
    float innovDragY;
    float innovSideslip;
};


// @LoggerMessage: XKF3
// @Description: EKF3 innovations
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: IVN: Innovation in velocity (North component)
// @Field: IVE: Innovation in velocity (East component)
// @Field: IVD: Innovation in velocity (Down component)
// @Field: IPN: Innovation in position (North component)
// @Field: IPE: Innovation in position (East component)
// @Field: IPD: Innovation in position (Down component)
// @Field: IMX: Innovation in magnetic field strength (X-axis component)
// @Field: IMY: Innovation in magnetic field strength (Y-axis component)
// @Field: IMZ: Innovation in magnetic field strength (Z-axis component)
// @Field: IYAW: Innovation in vehicle yaw
// @Field: IVT: Innovation in true-airspeed
// @Field: RErr: Accumulated relative error of this core with respect to active primary core
// @Field: ErSc: A consolidated error score where higher numbers are less healthy

/**
 * @brief EKF3 measurement innovations (residuals between predicted and measured values)
 * 
 * @details Innovations are the cornerstone of Kalman filter health monitoring. Each innovation
 *          represents the difference between what the EKF predicted a sensor would measure
 *          and what it actually measured:
 *          
 *          Innovation = Measurement - Prediction
 * 
 *          Small innovations indicate good filter performance (predictions match reality).
 *          Large innovations indicate:
 *          - Sensor malfunction or interference
 *          - Incorrect EKF modeling (e.g., wrong noise parameters)
 *          - Vehicle dynamics exceeding EKF assumptions
 *          - External disturbances (e.g., GPS multipath, magnetic interference)
 * 
 *          This message includes innovations for:
 *          - GPS velocity (NED frame, cm/s scaled to int16_t)
 *          - GPS position (NED frame, cm scaled to int16_t)
 *          - Magnetometer (body frame, milligauss scaled to int16_t)
 *          - Yaw (centidegrees scaled to int16_t)
 *          - Airspeed (cm/s scaled to int16_t)
 * 
 *          Additionally logs:
 *          - RErr: Divergence metric for multi-EKF lane switching
 *          - ErSc: Composite health score combining all innovation magnitudes
 * 
 * @note Essential for post-flight analysis of navigation failures. Spikes in innovations
 *       correlate with sensor issues or adverse flight conditions. Compare with XKF4 variances
 *       to determine if innovations exceeded expected statistical bounds (innovation gate failures).
 * 
 * @see XKF4 for innovation variance and test ratio information
 */
struct PACKED log_XKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovYaw;
    int16_t innovVT;
    float rerr;
    float errorScore;
};


// @LoggerMessage: XKF4
// @Description: EKF3 variances.  SV, SP, SH and SM are probably best described as 'Squared Innovation Test Ratios' where values <1 tells us the measurement was accepted and >1 tells us it was rejected. They represent the square of the (innovation / maximum allowed innovation) where the innovation is the difference between predicted and measured value and the maximum allowed innovation is determined from the uncertainty of the measurement, uncertainty of the prediction and scaled using the number of standard deviations set by the innovation gate parameter for that measurement, eg EK3_MAG_I_GATE, EK3_HGT_I_GATE, etc
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: SV: Square root of the velocity variance
// @Field: SP: Square root of the position variance
// @Field: SH: Square root of the height variance
// @Field: SM: Magnetic field variance
// @Field: SVT: Square root of the total airspeed variance
// @Field: errRP: Filtered error in roll/pitch estimate
// @Field: OFN: Most recent position reset (North component)
// @Field: OFE: Most recent position reset (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement, 5:drag measurement)
// @Field: SS: Filter solution status
// @FieldBitmaskEnum: SS: NavFilterStatusBit
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index

/**
 * @brief EKF3 state variances, innovation test ratios, and filter health status
 * 
 * @details This message provides comprehensive EKF health diagnostics:
 * 
 *          **Variance Fields (uncertainty estimates)**:
 *          - SV, SP, SH, SVT: Square root of state covariance diagonal elements
 *          - Represent 1-sigma (68% confidence) uncertainty bounds
 *          - Values increase with poor sensor quality or dynamic maneuvering
 *          - Values decrease as sensors converge and filter stabilizes
 *          - SM: Actually contains squared innovation test ratio for magnetometer (see description)
 * 
 *          **Innovation Test Ratios** (measurement acceptance/rejection):
 *          Despite field names suggesting variance, SV/SP/SH/SM actually represent squared
 *          innovation test ratios during measurement updates:
 *          - Value < 1.0: Innovation within gate, measurement accepted
 *          - Value > 1.0: Innovation exceeded gate, measurement rejected
 *          - Formula: (innovation / gate_threshold)²
 *          - Gate thresholds set by EK3_*_I_GATE parameters (typically 3-5 sigma)
 * 
 *          **Status Fields**:
 *          - errRP: Roll/pitch tilt error magnitude (radians)
 *          - OFN/OFE: Position reset applied when GPS glitch detected or EKF reinitialized
 *          - FS: Fault bitmask indicating detected sensor failures
 *          - TS: Timeout bitmask indicating sensors not providing recent measurements
 *          - SS: Solution status bitmask (using NavFilterStatusBit enum)
 *          - GPS: GPS quality metrics and fix type
 *          - PI: Which EKF core is currently active (primary) for flight control
 * 
 * @note Critical for diagnosing navigation failures. Position resets (OFN/OFE non-zero) indicate
 *       GPS glitches or EKF reinitialization events. Timeout bits indicate missing sensor data.
 *       Fault bits indicate rejected sensors. High variances indicate low confidence in estimates.
 * 
 * @warning Multi-EKF configurations run 2-3 parallel filters. The PI (primary index) field indicates
 *          which core is currently trusted for flight control. Lane switches occur when a core's
 *          errorScore (XKF3) becomes significantly better than the current primary.
 */
struct PACKED log_XKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarM;
    int16_t sqrtvarVT;
    float   tiltErr;
    float  offsetNorth;
    float  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint32_t solution;
    uint16_t gps;
    int8_t primary;
};


// @LoggerMessage: XKF5
// @Description: EKF3 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: NI: Normalised flow variance
// @Field: FIX: Optical flow LOS rate vector innovations from the main nav filter (X-axis)
// @Field: FIY: Optical flow LOS rate vector innovations from the main nav filter (Y-axis)
// @Field: AFI: Optical flow LOS rate innovation from terrain offset estimator
// @Field: HAGL: Height above ground level
// @Field: offset: Estimated vertical position of the terrain relative to the nav filter zero datum
// @Field: RI: Range finder innovations
// @Field: rng: Measured range
// @Field: Herr: Filter ground offset state error
// @Field: eAng: Magnitude of angular error
// @Field: eVel: Magnitude of velocity error
// @Field: ePos: Magnitude of position error

/**
 * @brief EKF3 optical flow, rangefinder innovations, and consolidated error metrics
 * 
 * @details This message combines terrain-relative sensor diagnostics with overall filter error magnitudes:
 * 
 *          **Optical Flow Data**:
 *          - FIX/FIY: Flow innovation in body X/Y axes (rad/s scaled to int16_t)
 *          - AFI: Terrain estimator flow innovation (rad/s scaled to int16_t)
 *          - NI: Normalized flow variance for quality assessment (dimensionless)
 *          - Used for GPS-denied navigation and precision landing
 * 
 *          **Rangefinder/Terrain Data**:
 *          - RI: Range innovation (measured - predicted) in cm
 *          - rng: Raw rangefinder measurement in cm
 *          - HAGL: Estimated height above ground level in cm
 *          - offset: Terrain height relative to EKF origin in cm
 *          - Herr: Terrain estimation uncertainty in cm
 * 
 *          **Consolidated Error Magnitudes**:
 *          - eAng: Total angular error magnitude combining roll/pitch/yaw (radians)
 *          - eVel: Total velocity error magnitude combining N/E/D components (m/s)
 *          - ePos: Total position error magnitude combining N/E/D components (m)
 *          - Provide single-number health metrics for quick assessment
 * 
 *          Optical flow measures ground-relative motion using downward camera, enabling position hold
 *          without GPS. Rangefinder provides direct height-above-ground measurement for terrain following
 *          and precision landing. Both are integrated with GPS/IMU in the main EKF for robust navigation.
 * 
 * @note "General dumping ground" refers to this message's historical use for miscellaneous diagnostic
 *       data that didn't fit cleanly into other message types. The consolidated error magnitudes
 *       (eAng, eVel, ePos) provide quick health assessment without analyzing individual state variances.
 * 
 * @see XKF3 for individual sensor innovations, XKF4 for detailed variance information
 */
struct PACKED log_XKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
    float angErr;
    float velErr;
    float posErr;
};


// @LoggerMessage: XKFD
// @Description: EKF3 Body Frame Odometry errors
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: IX: Innovation in velocity (X-axis)
// @Field: IY: Innovation in velocity (Y-axis)
// @Field: IZ: Innovation in velocity (Z-axis)
// @Field: IVX: Variance in velocity (X-axis)
// @Field: IVY: Variance in velocity (Y-axis)
// @Field: IVZ: Variance in velocity (Z-axis)

/**
 * @brief Body-frame odometry velocity innovations and variances
 * 
 * @details Records measurement residuals for body-frame velocity sensors such as:
 *          - Visual odometry from cameras (e.g., Intel RealSense T265)
 *          - Wheel odometry from encoders (ground vehicles)
 *          - Optical flow integration
 *          - DVL (Doppler Velocity Log) for underwater vehicles
 * 
 *          Unlike GPS which provides earth-frame (NED) velocity, these sensors measure velocity
 *          in the vehicle body frame (forward/right/down). Body-frame measurements are particularly
 *          useful for:
 *          - Indoor/GPS-denied navigation
 *          - Precision hovering and slow-speed maneuvering
 *          - Operations where GPS is unavailable or unreliable
 * 
 *          Fields use full float precision (not scaled) as body-frame velocity innovations are
 *          critical for camera-based navigation accuracy. Variances indicate measurement quality
 *          and are used to weight the sensor fusion appropriately.
 * 
 * @note Only logged when body-frame odometry sensors are configured and actively providing data.
 *       Common with AP_VisualOdom library integration. Typical logging rate: 10-30 Hz.
 * 
 * @see AP_VisualOdom for visual odometry sensor integration
 */
struct PACKED log_XKFD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float velInnovX;
    float velInnovY;
    float velInnovZ;
    float velInnovVarX;
    float velInnovVarY;
    float velInnovVarZ;
};

// @LoggerMessage: XKT
// @Description: EKF3 timing information
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: Cnt: count of samples used to create this message
// @Field: IMUMin: smallest IMU sample interval
// @Field: IMUMax: largest IMU sample interval
// @Field: EKFMin: low-passed achieved average time step rate for the EKF (minimum)
// @Field: EKFMax: low-passed achieved average time step rate for the EKF (maximum)
// @Field: AngMin: accumulated measurement time interval for the delta angle (minimum)
// @Field: AngMax: accumulated measurement time interval for the delta angle (maximum)
// @Field: VMin: accumulated measurement time interval for the delta velocity (minimum)
// @Field: VMax: accumulated measurement time interval for the delta velocity (maximum)

/**
 * @brief EKF3 timing diagnostics for IMU sample rates and filter update intervals
 * 
 * @details Records timing statistics essential for diagnosing CPU performance and IMU configuration:
 * 
 *          **IMU Sample Timing**:
 *          - IMUMin/IMUMax: Range of time intervals between raw IMU samples (seconds)
 *          - AngMin/AngMax: Range of delta-angle integration periods (seconds)
 *          - VMin/VMax: Range of delta-velocity integration periods (seconds)
 *          - Ideal: Consistent timing with Min ≈ Max (indicates stable sample rate)
 *          - Problem indicators: Large Min-Max spread suggests timing jitter or CPU overload
 * 
 *          **EKF Update Timing**:
 *          - EKFMin/EKFMax: Range of achieved EKF update rates (seconds per update)
 *          - Low-pass filtered to show sustained timing, not instantaneous
 *          - Should match target EKF rate (typically 10-50 Hz depending on platform)
 * 
 *          **Sample Counting**:
 *          - Cnt: Number of IMU samples accumulated in this logging period
 *          - Used to calculate statistics and detect dropped samples
 * 
 *          IMUs typically provide delta-angle (integrated gyro) and delta-velocity (integrated accel)
 *          over a measurement interval rather than instantaneous readings. These intervals must be
 *          known accurately for correct EKF integration. Timing variations directly affect state
 *          estimation accuracy and can cause filter divergence if excessive.
 * 
 * @note Critical for debugging EKF performance issues. Consistent timing (Min≈Max) is essential
 *       for optimal performance. Timing jitter > 10% indicates CPU overload or IMU driver issues.
 * 
 * @warning CPU overload causing timing variations can lead to navigation failures. If Min/Max spread
 *          is large, reduce logging rates, disable expensive features, or upgrade to faster hardware.
 */
struct PACKED log_XKT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint32_t timing_count;
    float dtIMUavg_min;
    float dtIMUavg_max;
    float dtEKFavg_min;
    float dtEKFavg_max;
    float delAngDT_min;
    float delAngDT_max;
    float delVelDT_min;
    float delVelDT_max;
};


// @LoggerMessage: XKFM
// @Description: EKF3 diagnostic data for on-ground-and-not-moving check
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: OGNM: True of on ground and not moving
// @Field: GLR: Gyroscope length ratio
// @Field: ALR: Accelerometer length ratio
// @Field: GDR: Gyroscope rate of change ratio
// @Field: ADR: Accelerometer rate of change ratio

/**
 * @brief EKF3 on-ground-not-moving detector diagnostic data
 * 
 * @details The EKF uses an "on ground and not moving" (OGNM) detector to:
 *          - Enable gyro bias learning while stationary
 *          - Inhibit GPS glitch detection during initialization
 *          - Manage state covariance growth appropriately for static vs dynamic conditions
 *          - Optimize filter parameters for different operational states
 * 
 *          Detection algorithm uses four statistical measures:
 * 
 *          **Gyroscope Length Ratio (GLR)**:
 *          - Ratio of current gyro magnitude to moving-threshold
 *          - Values < 1.0 indicate rotation below movement detection threshold
 *          - Detects vehicle rotational motion
 * 
 *          **Accelerometer Length Ratio (ALR)**:
 *          - Ratio of current accel magnitude deviation from gravity to threshold
 *          - Values < 1.0 indicate near-zero vibration and linear acceleration
 *          - Detects vehicle translation and vibration
 * 
 *          **Gyroscope Diff Ratio (GDR)**:
 *          - Ratio of gyro rate-of-change to threshold
 *          - Values < 1.0 indicate steady (non-changing) rotation rate
 *          - Detects angular acceleration
 * 
 *          **Accelerometer Diff Ratio (ADR)**:
 *          - Ratio of accel rate-of-change to threshold
 *          - Values < 1.0 indicate steady (non-changing) acceleration
 *          - Detects linear jerk
 * 
 *          OGNM flag is set true only when ALL four ratios remain below 1.0 for a sustained period
 *          (typically several seconds), ensuring the vehicle is genuinely stationary and not just
 *          momentarily hovering or experiencing temporary low motion.
 * 
 * @note Useful for diagnosing why EKF may not be learning gyro biases (requires OGNM true) or why
 *       GPS glitches are being detected during initialization (OGNM should be true at startup).
 * 
 * @see NavEKF3_core::setOnGroundAndNotMoving() for detector implementation
 */
struct PACKED log_XKFM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t ongroundnotmoving;
    float gyro_length_ratio;
    float accel_length_ratio;
    float gyro_diff_ratio;
    float accel_diff_ratio;
};


// @LoggerMessage: XKQ
// @Description: EKF3 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term

/**
 * @brief EKF3 attitude quaternion representation
 * 
 * @details Provides the full-precision quaternion representation of vehicle attitude, complementing
 *          the scaled Euler angles in XKF1. Quaternions are the internal EKF attitude representation
 *          because they:
 *          - Avoid gimbal lock at ±90° pitch
 *          - Enable efficient attitude integration and updates
 *          - Provide continuous (no wrap-around) representation
 *          - Support direct composition and interpolation
 * 
 *          Quaternion represents rotation from NED (North-East-Down) reference frame to body frame:
 *          - Q = [q1, q2, q3, q4] where q4 is the scalar (real) component
 *          - Normalized: q1² + q2² + q3² + q4² = 1.0
 *          - Convention: Hamilton (not JPL), right-handed rotation
 *          - Passive rotation (transforms vectors from NED to body frame)
 * 
 *          All four components use full float precision (not scaled) to maintain the numerical
 *          accuracy required for quaternion operations. Even small denormalization accumulates
 *          and causes attitude drift if quaternions are stored with reduced precision.
 * 
 *          Relationship to Euler angles (XKF1):
 *          - Roll: atan2(2*(q4*q1 + q2*q3), 1 - 2*(q1² + q2²))
 *          - Pitch: asin(2*(q4*q2 - q3*q1))
 *          - Yaw: atan2(2*(q4*q3 + q1*q2), 1 - 2*(q2² + q3²))
 * 
 * @note Use quaternion logs for attitude analysis requiring full precision or at high pitch angles
 *       where Euler singularities occur. For routine analysis, XKF1 Euler angles are more intuitive.
 * 
 * @see XKF1 for Euler angle representation (roll, pitch, yaw)
 * @see AP_Math/quaternion.h for quaternion math operations
 */
struct PACKED log_XKQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float q1;
    float q2;
    float q3;
    float q4;
};


// @LoggerMessage: XKFS
// @Description: EKF3 sensor selection
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: MI: compass selection index
// @Field: BI: barometer selection index
// @Field: GI: GPS selection index
// @Field: AI: airspeed selection index
// @Field: SS: Source Set (primary=0/secondary=1/tertiary=2)
// @Field: GPS_GTA: GPS good to align
// @Field: GPS_CHK_WAIT: Waiting for GPS checks to pass
// @Field: MAG_FUSION: Magnetometer fusion (0=not fusing/1=fuse yaw/2=fuse mag)

/**
 * @brief EKF3 sensor selection indices and fusion status
 * 
 * @details Tracks which physical sensors are actively used by each EKF core. ArduPilot supports
 *          multiple instances of each sensor type (e.g., GPS1, GPS2, Compass1-3, Baro1-3) for
 *          redundancy. The EKF can:
 *          - Use a single "best" sensor from each type
 *          - Blend measurements from multiple sensors
 *          - Switch between sensors when failures are detected
 * 
 *          **Sensor Selection Indices**:
 *          - MI (Magnetometer Index): Which compass (0-3) is used for yaw/mag fusion
 *          - BI (Barometer Index): Which barometer (0-2) is used for altitude
 *          - GI (GPS Index): Which GPS receiver (0-1) is used for position/velocity
 *          - AI (Airspeed Index): Which airspeed sensor (0-1) is used (fixed-wing only)
 * 
 *          **Source Set**:
 *          - ArduPilot supports up to 3 source sets (primary/secondary/tertiary)
 *          - Each set defines a consistent combination of sensors
 *          - EKF can switch source sets if primary set fails health checks
 *          - SS field: 0=primary, 1=secondary, 2=tertiary
 * 
 *          **GPS Alignment Status**:
 *          - GPS_GTA: GPS quality sufficient to initialize/align EKF (boolean)
 *          - GPS_CHK_WAIT: Waiting for GPS checks to pass before using (boolean)
 *          - EKF requires good GPS (hdop, satellite count, movement) before first use
 * 
 *          **Magnetometer Fusion Mode**:
 *          - MAG_FUSION: 0=not using magnetometer, 1=fusing yaw only, 2=fusing full 3D field
 *          - Mode 1: Uses mag for yaw but not for position (reduces mag interference effects)
 *          - Mode 2: Full 3D magnetometer fusion (better heading accuracy in benign environments)
 *          - Mode 0: GPS yaw or no mag (indoor flight, mag disabled, or compass failure)
 * 
 * @note Essential for diagnosing sensor redundancy behavior. Sensor index changes indicate automatic
 *       failover due to detected problems. Compare with XKF4 fault/timeout bits to understand why
 *       sensors were rejected.
 * 
 * @see AP_AHRS for source set management
 * @see EKF3_SRC* parameters for configuring source sets
 */
struct PACKED log_XKFS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t mag_index;
    uint8_t baro_index;
    uint8_t gps_index;
    uint8_t airspeed_index;
    uint8_t source_set;
    uint8_t gps_good_to_align;
    uint8_t wait_for_gps_checks;
    uint8_t mag_fusion;
};

// @LoggerMessage: XKTV
// @Description: EKF3 Yaw Estimator States
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: TVS: Tilt Error Variance from symbolic equations (rad^2)
// @Field: TVD: Tilt Error Variance from difference method (rad^2)

/**
 * @brief EKF3 GSF (Gaussian Sum Filter) yaw estimator variance diagnostics
 * 
 * @details The EKF3 includes a specialized Gaussian Sum Filter for yaw initialization in challenging
 *          conditions where magnetometer is unavailable, unreliable, or disabled. The GSF runs multiple
 *          parallel yaw hypotheses and converges to the correct yaw through GPS velocity observations
 *          during vehicle movement.
 * 
 *          This message logs tilt error variance estimates computed by two independent methods:
 * 
 *          **TVS (Symbolic Equations Method)**:
 *          - Tilt variance derived from symbolic differentiation of measurement equations
 *          - Analytically computed from state covariances and Jacobian matrices
 *          - Theoretically correct under linear approximation assumptions
 *          - Units: rad² (variance of tilt error)
 * 
 *          **TVD (Difference Method)**:
 *          - Tilt variance estimated from empirical differences between filter hypotheses
 *          - Based on actual spread of the parallel yaw estimates
 *          - Captures nonlinear effects and real-world variance
 *          - Units: rad² (variance of tilt error)
 * 
 *          **Comparison Usage**:
 *          - TVS and TVD should agree when filter is operating correctly
 *          - Large TVS >> TVD indicates analytical model mismatch with reality
 *          - Large TVD >> TVS indicates unexpected measurement noise or dynamics
 *          - Both decreasing indicates yaw hypothesis convergence during flight
 * 
 *          The GSF is particularly valuable for:
 *          - Fixed-wing launch where initial yaw is unknown
 *          - Indoor flight with magnetometer disabled
 *          - Environments with severe magnetic distortion
 *          - Recovery from magnetometer failures during flight
 * 
 * @note Only logged when GSF yaw estimator is active (EK3_GSF_USE_MASK parameter).
 *       Tilt error variance decreases as vehicle maneuvers provide yaw observability through
 *       GPS velocity measurements. Sustained straight flight provides poor yaw observability.
 * 
 * @see NavEKF3_core::runYawEstimatorPrediction() for GSF implementation
 */
struct PACKED log_XKTV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float tvs;
    float tvd;
};

// @LoggerMessage: XKV1
// @Description: EKF3 State variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: V00: Variance for state 0 (attitude quaternion)
// @Field: V01: Variance for state 1 (attitude quaternion)
// @Field: V02: Variance for state 2 (attitude quaternion)
// @Field: V03: Variance for state 3 (attitude quaternion)
// @Field: V04: Variance for state 4 (velocity-north)
// @Field: V05: Variance for state 5 (velocity-east)
// @Field: V06: Variance for state 6 (velocity-down)
// @Field: V07: Variance for state 7 (position-north)
// @Field: V08: Variance for state 8 (position-east)
// @Field: V09: Variance for state 9 (position-down)
// @Field: V10: Variance for state 10 (delta-angle-bias-x)
// @Field: V11: Variance for state 11 (delta-angle-bias-y)

// @LoggerMessage: XKV2
// @Description: more EKF3 State Variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: V12: Variance for state 12 (delta-angle-bias-z)
// @Field: V13: Variance for state 13 (delta-velocity-bias-x)
// @Field: V14: Variance for state 14 (delta-velocity-bias-y)
// @Field: V15: Variance for state 15 (delta-velocity-bias-z)
// @Field: V16: Variance for state 16 (Earth-frame mag-field-bias-x)
// @Field: V17: Variance for state 17 (Earth-frame mag-field-bias-y)
// @Field: V18: Variance for state 18 (Earth-frame mag-field-bias-z)
// @Field: V19: Variance for state 19 (body-frame mag-field-bias-x)
// @Field: V20: Variance for state 20 (body-frame mag-field-bias-y)
// @Field: V21: Variance for state 21 (body-frame mag-field-bias-z)
// @Field: V22: Variance for state 22 (wind-north)
// @Field: V23: Variance for state 23 (wind-east)

/**
 * @brief Complete EKF3 state covariance diagonal elements (split across XKV1 and XKV2 messages)
 * 
 * @details The Extended Kalman Filter maintains a state vector and covariance matrix representing
 *          the estimate and its uncertainty. This structure logs the diagonal elements of the
 *          covariance matrix, which represent the variance (squared uncertainty) of each state:
 * 
 *          **State Vector Organization** (24 states total):
 *          - States 0-3: Attitude quaternion error components (rad²)
 *          - States 4-6: Velocity NED (m²/s²)
 *          - States 7-9: Position NED (m²)
 *          - States 10-12: Gyro delta-angle bias XYZ (rad²)
 *          - States 13-15: Accel delta-velocity bias XYZ (m²/s²)
 *          - States 16-18: Earth magnetic field bias NED (mG²)
 *          - States 19-21: Body magnetic field bias XYZ (mG²)
 *          - States 22-23: Wind velocity NE (m²/s²)
 * 
 *          **Message Split Rationale**:
 *          Due to binary log message size constraints, the 24 variances are split across two
 *          messages (XKV1 and XKV2), each carrying 12 float values plus header/timestamp/core.
 *          This maintains compatibility with existing logging infrastructure while providing
 *          complete state covariance visibility.
 * 
 *          **Variance Interpretation**:
 *          - Variance = σ² (square of standard deviation)
 *          - √Variance = 1-sigma uncertainty bound (68% confidence)
 *          - Small variance: High confidence in state estimate
 *          - Large variance: Low confidence, needs better sensor data or tuning
 *          - Growing variance: Filter divergence or insufficient sensor updates
 *          - Shrinking variance: Filter convergence as sensors provide information
 * 
 *          **Usage in Analysis**:
 *          - Initial variances reflect EK3_*_P_NOISE parameter settings (process noise)
 *          - Variances evolve according to filter dynamics and sensor measurements
 *          - Compare with XKF4 aggregate variances (SV, SP, SH) which are derived from these
 *          - Essential for advanced EKF tuning and algorithm development
 *          - Diagnose which states are poorly observable (persistently high variance)
 * 
 * @note Full state covariance logging has significant storage overhead (2 messages × 13 fields × 4 bytes
 *       = 104 bytes per logging cycle). Typically enabled only for detailed analysis or algorithm
 *       development. For routine operations, XKF4 aggregate variances are sufficient.
 * 
 * @see XKF4 for aggregate variance measures and innovation test ratios
 * @see EK3_*_P_NOISE parameters for process noise tuning (affects variance growth)
 * @see NavEKF3_core::CovariancePrediction() for covariance propagation implementation
 */
struct PACKED log_XKV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float v00;
    float v01;
    float v02;
    float v03;
    float v04;
    float v05;
    float v06;
    float v07;
    float v08;
    float v09;
    float v10;
    float v11;
};

/**
 * @brief Conditional compilation guard for EKF3 feature availability
 * 
 * @details HAL_NAVEKF3_AVAILABLE is a compile-time feature flag that determines whether the
 *          NavEKF3 library is included in the firmware build. This flag is controlled by:
 *          - Board hardware capabilities (RAM, flash, CPU speed)
 *          - Vehicle-specific feature requirements
 *          - hwdef (hardware definition) files for each board
 *          - Global feature configuration settings
 * 
 *          When HAL_NAVEKF3_AVAILABLE is 0 (disabled):
 *          - NavEKF3 library code is not compiled or linked
 *          - LOG_STRUCTURE_FROM_NAVEKF3 expands to empty (no log structures registered)
 *          - Firmware falls back to EKF2, DCM, or external AHRS
 *          - Binary size reduced by ~100KB (significant for small boards)
 * 
 *          When HAL_NAVEKF3_AVAILABLE is 1 (enabled):
 *          - Full EKF3 functionality compiled and available
 *          - Log structures registered with AP_Logger for data recording
 *          - EKF3 can be selected via AHRS_EKF_TYPE parameter
 * 
 *          Typical scenarios where EKF3 is disabled:
 *          - Small F1/F4 microcontrollers with limited flash (<1MB)
 *          - Minimal builds for specific applications (antenna tracker, periph nodes)
 *          - Memory-constrained platforms where EKF2 is sufficient
 *          - Custom builds optimizing for binary size
 * 
 * @note Most modern ArduPilot boards (F7, H7 processors) have EKF3 enabled by default.
 *       Check your board's hwdef file or use "param show AHRS_EKF_TYPE" to determine availability.
 * 
 * @see AP_HAL_ChibiOS/hwdef for board-specific feature definitions
 * @see AHRS_EKF_TYPE parameter to select between EKF2 (2) and EKF3 (3)
 */
#if HAL_NAVEKF3_AVAILABLE

/**
 * @brief EKF3 log structure registration macro for integration with AP_Logger
 * 
 * @details This macro expands to a complete set of log message structure definitions that are
 *          incorporated into the system-wide logging configuration. Each entry in the macro
 *          defines a log message type with the following components:
 * 
 *          **Structure Definition Format**:
 *          { LOG_*_MSG, sizeof(log_*), "NAME", "FORMAT", "LABELS", "UNITS", "MULTIPLIERS", streaming }
 * 
 *          **Field Descriptions**:
 *          1. Message ID: LOG_XKF*_MSG enumeration constant
 *          2. Size: sizeof(log_*) for memory allocation and validation
 *          3. Name: 4-character message name (e.g., "XKF1") for log readers
 *          4. Format: Type specification string defining field sizes and encodings:
 *             - Q: uint64_t (8 bytes) - typically time_us timestamp
 *             - B: uint8_t (1 byte) - flags, indices, booleans
 *             - b: int8_t (1 byte) - signed small integers
 *             - H: uint16_t (2 bytes) - unsigned short integers
 *             - h: int16_t (2 bytes) - signed short integers (common for scaled values)
 *             - I: uint32_t (4 bytes) - unsigned integers
 *             - i: int32_t (4 bytes) - signed integers
 *             - f: float (4 bytes) - IEEE-754 single precision
 *             - d: double (8 bytes) - IEEE-754 double precision (rarely used)
 *             - c: int16_t (2 bytes) - legacy centidegree/centimeter scaling
 *             - C: uint16_t (2 bytes) - legacy unsigned scaling
 *          5. Labels: Comma-separated field names matching structure members
 *          6. Units: Single-character unit codes (s=seconds, m=meters, n=m/s, d=degrees, etc.)
 *          7. Multipliers: Scaling indicators (F=format, B=base, 0=no scaling, etc.)
 *          8. Streaming: Boolean indicating if message should be streamed via MAVLink telemetry
 * 
 *          **Unit Encoding**:
 *          - s: seconds
 *          - d: degrees
 *          - k: degrees Kelvin
 *          - r: radians
 *          - m: meters
 *          - n: meters/second
 *          - N: newton
 *          - G: gauss (magnetic field)
 *          - o: ohm
 *          - A: ampere
 *          - V: volt
 *          - v: volt/cell
 *          - ?: unknown/dimensionless
 *          - -: no unit
 * 
 *          **Multiplier Encoding**:
 *          - F: Format-specified (no additional scaling)
 *          - B: Base unit (1:1 scaling)
 *          - 0: Not scaled/not applicable
 *          - C: Centi scale (×100 in log, ×0.01 in real units)
 *          - ?: Unknown scaling
 * 
 *          **Integration with AP_Logger**:
 *          This macro is included in the global LOG_STRUCTURE_FROM_* aggregation in
 *          AP_Logger/LogStructure.h. During system initialization:
 *          1. AP_Logger allocates message buffers sized by sizeof(log_*)
 *          2. Format strings enable ground stations to parse binary logs
 *          3. Unit/multiplier metadata enables automatic scaling in analysis tools
 *          4. Streaming flag determines if messages go to MAVLink in addition to SD card
 * 
 * @note Binary log format compatibility is critical. Ground stations (Mission Planner, QGroundControl,
 *       MAVExplorer) parse logs using these format strings. Changes to field order, types, or count
 *       break compatibility with existing tools and historical log analysis.
 * 
 * @warning When modifying log structures:
 *          - Never change existing message field order or types (breaks historical log parsing)
 *          - Never reuse message names for different structures (confuses ground stations)
 *          - Create new message types (e.g., XKF6, XKF7) for new data instead of modifying existing
 *          - Update ground station message definitions in sync with firmware changes
 *          - Test with multiple ground station software before releasing
 *          - Document changes in release notes for log analysis tool developers
 * 
 * @see AP_Logger/LogStructure.h for the global log structure registration system
 * @see AP_Logger/LogStructure.cpp for format string documentation and parsing code
 * @see Tools/Replay for log replay system that validates message formats
 * @see https://ardupilot.org/dev/docs/code-overview-dataflash.html for logging system documentation
 * 
 * Source: libraries/AP_NavEKF3/LogStructure.h:432-459
 */
#define LOG_STRUCTURE_FROM_NAVEKF3        \
    { LOG_XKF0_MSG, sizeof(log_XKF0), \
      "XKF0","QBBccCCcccccccc","TimeUS,C,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD", "s#-m---mmmmmmmm", "F--B---BBBBBBBB" , true }, \
    { LOG_XKF1_MSG, sizeof(log_XKF1), \
      "XKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" , true }, \
    { LOG_XKF2_MSG, sizeof(log_XKF2), \
      "XKF2","QBccccchhhhhhfff","TimeUS,C,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,IDX,IDY,IS", "s#---nnGGGGGGoor", "F----BBCCCCCC000" , true }, \
    { LOG_XKF3_MSG, sizeof(log_XKF3), \
      "XKF3","QBcccccchhhccff","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT,RErr,ErSc", "s#nnnmmmGGGd?--", "F-BBBBBBCCCBB00" , true }, \
    { LOG_XKF4_MSG, sizeof(log_XKF4), \
      "XKF4","QBcccccfffHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------mm-----", "F-------??-----" , true }, \
    { LOG_XKF5_MSG, sizeof(log_XKF5), \
      "XKF5","QBBhhhcccCCfff","TimeUS,C,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos", "s#----m???mrnm", "F-----BBBBB000" , true }, \
    { LOG_XKFD_MSG, sizeof(log_XKFD), \
      "XKFD","QBffffff","TimeUS,C,IX,IY,IZ,IVX,IVY,IVZ", "s#------", "F-------" , true }, \
    { LOG_XKFM_MSG, sizeof(log_XKFM),   \
      "XKFM", "QBBffff", "TimeUS,C,OGNM,GLR,ALR,GDR,ADR", "s#-----", "F------", true }, \
    { LOG_XKFS_MSG, sizeof(log_XKFS), \
      "XKFS","QBBBBBBBBB","TimeUS,C,MI,BI,GI,AI,SS,GPS_GTA,GPS_CHK_WAIT,MAG_FUSION", "s#--------", "F---------" , true }, \
    { LOG_XKQ_MSG, sizeof(log_XKQ), "XKQ", "QBffff", "TimeUS,C,Q1,Q2,Q3,Q4", "s#????", "F-????" , true }, \
    { LOG_XKT_MSG, sizeof(log_XKT),   \
      "XKT", "QBIffffffff", "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax", "s#sssssssss", "F-000000000", true }, \
    { LOG_XKTV_MSG, sizeof(log_XKTV),                         \
      "XKTV", "QBff", "TimeUS,C,TVS,TVD", "s#rr", "F-00", true }, \
    { LOG_XKV1_MSG, sizeof(log_XKV), \
      "XKV1","QBffffffffffff","TimeUS,C,V00,V01,V02,V03,V04,V05,V06,V07,V08,V09,V10,V11", "s#------------", "F-------------" , true }, \
    { LOG_XKV2_MSG, sizeof(log_XKV), \
      "XKV2","QBffffffffffff","TimeUS,C,V12,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23", "s#------------", "F-------------" , true },

/**
 * @brief Empty macro definition when EKF3 is not available
 * 
 * @details When HAL_NAVEKF3_AVAILABLE is 0 (false), this branch defines LOG_STRUCTURE_FROM_NAVEKF3
 *          as an empty macro. This allows the logging system's structure aggregation to work correctly
 *          regardless of whether EKF3 is compiled into the firmware.
 * 
 *          Effect of empty macro:
 *          - No EKF3 log messages registered with AP_Logger
 *          - No EKF3 message IDs allocated in the log message enumeration
 *          - Reduces logging system memory footprint
 *          - Ground stations will not see EKF3 log entries in recorded logs
 *          - Log analysis tools can still parse logs from EKF3-enabled builds
 * 
 *          This pattern is used throughout ArduPilot for optional subsystems, ensuring that
 *          feature flags cleanly enable/disable entire modules without requiring extensive
 *          #ifdef blocks throughout the codebase.
 * 
 * @note The macro must still be defined (even if empty) because LOG_STRUCTURE_FROM_NAVEKF3
 *       is referenced in AP_Logger's structure aggregation regardless of EKF3 availability.
 * 
 * @see AP_Logger/LogStructure.h for how optional log structures are aggregated
 */
#else
  #define LOG_STRUCTURE_FROM_NAVEKF3
#endif // HAL_NAVEKF3_AVAILABLE
