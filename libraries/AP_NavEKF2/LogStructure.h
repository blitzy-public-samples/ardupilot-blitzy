/**
 * @file LogStructure.h
 * @brief Defines binary log message structures for EKF2 telemetry and diagnostic data
 * 
 * @details This header file defines the packed binary log structures used by NavEKF2
 *          to record diagnostic telemetry data to the AP_Logger binary log system.
 *          
 *          The log structures capture comprehensive EKF2 state estimation data including:
 *          - Estimator outputs (attitude, velocity, position, gyro bias)
 *          - Sensor innovations and innovation test ratios
 *          - Variances and error metrics
 *          - Timing diagnostics
 *          - Beacon positioning data
 *          
 *          Log messages are written by AP_NavEKF2_Logging.cpp at various rates
 *          depending on the LOG_REPLAY parameter and logging configuration.
 *          
 *          Structure Naming Convention:
 *          - log_NKFx: EKF2 log structures (x = 0-5)
 *          - log_NKQ: EKF2 quaternion attitude
 *          - log_NKT: EKF2 timing diagnostics
 *          
 *          Integration with AP_Logger:
 *          The LOG_STRUCTURE_FROM_NAVEKF2 macro registers all log message definitions
 *          with the AP_Logger system, providing message IDs, sizes, field names,
 *          format strings, units, and multipliers for ground station log analysis tools.
 * 
 * @note All structures use PACKED attribute to ensure consistent binary layout
 *       across platforms for log file compatibility.
 * 
 * @warning Many fields use scaled integer formats (e.g., centidegrees, cm) to reduce
 *          log bandwidth. Always check units and scaling factors when analyzing logs.
 * 
 * @see AP_NavEKF2_Logging.cpp for log writing implementation
 * @see AP_Logger for binary logging system
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:1-308
 */

#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_AHRS/AP_AHRS_config.h>

/**
 * @def LOG_IDS_FROM_NAVEKF2
 * @brief Defines the set of log message IDs used by NavEKF2
 * 
 * @details This macro provides a comma-separated list of all log message type
 *          identifiers used by the NavEKF2 module. These IDs are registered
 *          with the AP_Logger system to enable log message parsing and analysis.
 *          
 *          Message ID Definitions:
 *          - LOG_NKF0_MSG: Beacon sensor diagnostics
 *          - LOG_NKF1_MSG: Primary estimator outputs (attitude, velocity, position)
 *          - LOG_NKF2_MSG: Secondary outputs (wind, mag field, gyro scale factors)
 *          - LOG_NKF3_MSG: Sensor innovations for fault detection
 *          - LOG_NKF4_MSG: Variances and filter status
 *          - LOG_NKF5_MSG: Optical flow and rangefinder innovations
 *          - LOG_NKQ_MSG: Attitude quaternion
 *          - LOG_NKT_MSG: Timing diagnostics
 *          - LOG_NKY0_MSG, LOG_NKY1_MSG: Yaw estimator data
 * 
 * @note This macro is used in log message enumeration and must be updated
 *       if new EKF2 log message types are added.
 */
#define LOG_IDS_FROM_NAVEKF2 \
    LOG_NKF0_MSG,  \
    LOG_NKF1_MSG,  \
    LOG_NKF2_MSG,  \
    LOG_NKF3_MSG,  \
    LOG_NKF4_MSG,  \
    LOG_NKF5_MSG,  \
    LOG_NKQ_MSG,   \
    LOG_NKT_MSG,   \
    LOG_NKY0_MSG,  \
    LOG_NKY1_MSG


/**
 * @struct log_NKF0
 * @brief EKF2 beacon sensor diagnostic data for positioning systems
 * 
 * @details This structure captures diagnostic telemetry for beacon-based positioning
 *          systems (e.g., Marvelmind, Pozyx) integrated with NavEKF2. Beacon systems
 *          provide absolute position measurements that can be fused with IMU data for
 *          GPS-denied navigation.
 *          
 *          Key diagnostic data includes:
 *          - Range measurements and innovations (difference between measured and predicted)
 *          - Innovation consistency test ratios for fault detection
 *          - Beacon position estimates in NED frame
 *          - Vertical offset estimates for beacon coordinate frame alignment
 *          
 *          Coordinate Frame: All position fields use NED (North-East-Down) frame
 *          relative to the EKF origin.
 *          
 *          Units and Scaling:
 *          - All distance/position fields: centimeters (cm) as int16_t
 *          - Test ratio: dimensionless, scaled by 100 (divide by 100 for actual ratio)
 *          
 *          Usage for Log Analysis:
 *          - Monitor testRatio (TR) field: values > 100 indicate innovation rejection
 *          - Check innovation (innov) magnitude for sensor health
 *          - Verify beacon positions (beaconPos*) are reasonable for known beacon layout
 * 
 * @note This log message is only written when beacon positioning is active and
 *       LOG_REPLAY parameter enables beacon logging.
 * 
 * @warning Innovation test ratio (testRatio) is critical for EKF health monitoring.
 *          Sustained high values (>100) may trigger EKF failsafe or lane switching.
 * 
 * @see AP_Beacon for beacon hardware interface
 * @see NavEKF2_core::FuseRngBcn() for beacon range fusion algorithm
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:36-53
 */
// @LoggerMessage: NKF0
// @Description: EKF2 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
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
// @Field: OFN: always zero
// @Field: OFE: always zero
// @Field: OFD: always zero
struct PACKED log_NKF0 {
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


/**
 * @struct log_NKF1
 * @brief EKF2 primary state estimates - attitude, velocity, position, and gyro bias
 * 
 * @details This structure contains the core navigation state estimates from NavEKF2,
 *          representing the filter's best estimate of vehicle attitude, velocity,
 *          position, and IMU biases. This is the primary telemetry for monitoring
 *          EKF2 performance and vehicle state.
 *          
 *          State Vector Components:
 *          - Attitude: Roll, pitch, yaw Euler angles
 *          - Velocity: 3D velocity vector in NED frame
 *          - Position: 3D position relative to EKF origin in NED frame
 *          - Gyro Bias: Estimated gyroscope bias corrections
 *          - Vertical Rate: Filtered vertical velocity for altitude control
 *          
 *          Coordinate Frames:
 *          - Attitude: Euler angles relative to NED frame
 *          - Velocity: NED (North-East-Down) frame in m/s
 *          - Position: NED frame in meters relative to EKF origin
 *          - Gyro bias: Body frame in deg/s
 *          
 *          Units and Scaling:
 *          - roll, pitch: centidegrees (multiply by 0.01 for degrees)
 *          - yaw: centidegrees (multiply by 0.01 for degrees)
 *          - velN, velE, velD: m/s (float)
 *          - posN, posE, posD: meters (float)
 *          - posD_dot: m/s (float) - vertical velocity estimate
 *          - gyrX, gyrY, gyrZ: deg/s * 100 (divide by 100 for deg/s)
 *          - originHgt: cm (centimeters)
 *          
 *          Usage for Flight Control:
 *          - This data feeds the AHRS interface used by attitude controllers
 *          - Position estimates drive navigation and position hold modes
 *          - Gyro bias estimates improve IMU accuracy over time
 * 
 * @note This log is written at high rate (typically 25-50 Hz) and is essential
 *       for flight log analysis and EKF replay.
 * 
 * @warning These estimates are the primary navigation source for flight control.
 *          Divergence from actual vehicle state can cause loss of control.
 *          Monitor innovation and variance logs (NKF3, NKF4) for filter health.
 * 
 * @see NavEKF2_core::quat2angle() for attitude conversion
 * @see NavEKF2_core::getVelNED() for velocity estimate interface
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:74-92
 */
// @LoggerMessage: NKF1
// @Description: EKF2 estimator outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
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
struct PACKED log_NKF1 {
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


/**
 * @struct log_NKF2
 * @brief EKF2 secondary state estimates - wind, magnetic field, and sensor corrections
 * 
 * @details This structure captures auxiliary state estimates from NavEKF2 including
 *          wind velocity, earth and body-frame magnetic field vectors, accelerometer
 *          bias, and gyroscope scale factor corrections.
 *          
 *          State Components:
 *          - Accelerometer Z-axis bias: Vertical acceleration offset correction
 *          - Gyro scale factors: Multiplicative corrections for gyro measurements
 *          - Wind velocity: 2D wind estimate in NED horizontal plane
 *          - Magnetic field: Both earth-frame (NED) and body-frame vectors
 *          
 *          Coordinate Frames:
 *          - Wind velocity: NED frame (North-East components only)
 *          - Magnetic field (MN, ME, MD): NED earth frame in milligauss
 *          - Magnetic field (MX, MY, MZ): Body frame in milligauss
 *          
 *          Units and Scaling:
 *          - AZbias: m/s² * 100 (divide by 100 for m/s²)
 *          - scaleX, scaleY, scaleZ: dimensionless scale factor * 100
 *          - windN, windE: cm/s (divide by 100 for m/s)
 *          - magN, magE, magD: milligauss
 *          - magX, magY, magZ: milligauss
 *          - index: Compass instance ID (0-based)
 *          
 *          Usage:
 *          - Wind estimates improve airspeed-based navigation for fixed-wing
 *          - Magnetic field estimates enable compass calibration validation
 *          - Gyro scale factors compensate for temperature-dependent gyro errors
 *          - Accelerometer bias improves altitude hold and vertical velocity accuracy
 * 
 * @note Wind estimation requires airspeed sensor or sufficient GPS motion.
 *       Gyro scale factor estimation requires EK2_ENABLE_EXTERNAL_NAV or similar.
 * 
 * @warning Magnetic field estimates are used for yaw determination. Large divergence
 *          between earth and body magnetic vectors may indicate compass calibration
 *          issues or magnetic interference.
 * 
 * @see NavEKF2_core::FuseAirspeed() for wind estimation
 * @see NavEKF2_core::FuseMagnetometer() for magnetic field fusion
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:112-129
 */
// @LoggerMessage: NKF2
// @Description: EKF2 estimator secondary outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: AZbias: Estimated accelerometer Z bias
// @Field: GSX: Gyro Scale Factor (X-axis)
// @Field: GSY: Gyro Scale Factor (Y-axis)
// @Field: GSZ: Gyro Scale Factor (Z-axis)
// @Field: VWN: Estimated wind velocity (moving-to-North component)
// @Field: VWE: Estimated wind velocity (moving-to-East component)
// @Field: MN: Magnetic field strength (North component)
// @Field: ME: Magnetic field strength (East component)
// @Field: MD: Magnetic field strength (Down component)
// @Field: MX: Magnetic field strength (body X-axis)
// @Field: MY: Magnetic field strength (body Y-axis)
// @Field: MZ: Magnetic field strength (body Z-axis)
// @Field: MI: Magnetometer used for data
struct PACKED log_NKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int8_t AZbias;
    int16_t scaleX;
    int16_t scaleY;
    int16_t scaleZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    uint8_t index;
};


/**
 * @struct log_NKF3
 * @brief EKF2 sensor innovations for fault detection and filter health monitoring
 * 
 * @details This structure records innovations (measurement residuals) for all sensors
 *          fused by NavEKF2. Innovations represent the difference between actual sensor
 *          measurements and the filter's predicted measurement based on current state
 *          estimates. Large innovations indicate either sensor faults or filter divergence.
 *          
 *          Innovation Types:
 *          - Velocity innovations: GPS or optical flow velocity vs predicted velocity
 *          - Position innovations: GPS position vs predicted position
 *          - Magnetic innovations: Compass reading vs predicted field from attitude/mag state
 *          - Yaw innovation: Compass-derived yaw vs EKF yaw estimate
 *          - Airspeed innovation: Pitot tube measurement vs predicted airspeed
 *          
 *          Coordinate Frames:
 *          - Velocity innovations (IVN, IVE, IVD): NED frame
 *          - Position innovations (IPN, IPE, IPD): NED frame
 *          - Magnetic innovations (IMX, IMY, IMZ): Body frame
 *          
 *          Units and Scaling:
 *          - innovVN, innovVE, innovVD: cm/s
 *          - innovPN, innovPE, innovPD: cm
 *          - innovMX, innovMY, innovMZ: milligauss
 *          - innovYaw: centidegrees
 *          - innovVT: cm/s (true airspeed innovation)
 *          - rerr: dimensionless relative error (float)
 *          - errorScore: dimensionless health metric (float, higher = less healthy)
 *          
 *          Fault Detection Usage:
 *          - Compare innovations to innovation variances (in NKF4)
 *          - Large sustained innovations trigger measurement rejection
 *          - Innovation sequence used for lane switching decisions
 *          - Error score determines active primary EKF core
 * 
 * @note Innovation magnitudes alone are not sufficient for fault detection - they
 *       must be normalized by innovation variance (see NKF4) to determine if the
 *       innovation is statistically significant.
 * 
 * @warning errorScore is critical for EKF health monitoring and primary core selection.
 *          Rapid increases in errorScore may indicate impending filter failure and
 *          trigger EKF failsafe actions. Monitor this field for early fault detection.
 * 
 * @see NavEKF2_core::SelectVelPosFusion() for innovation-based measurement selection
 * @see NavEKF2_core::calcIMU_Weighting() for error score calculation
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:149-166
 */
// @LoggerMessage: NKF3
// @Description: EKF2 innovations
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
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
struct PACKED log_NKF3 {
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


/**
 * @struct log_NKF4
 * @brief EKF2 state variances, filter status, and health metrics
 * 
 * @details This structure captures EKF2 uncertainty estimates (variances) and filter
 *          health status information. Variances represent the filter's confidence in
 *          its state estimates - low variance indicates high confidence, high variance
 *          indicates uncertainty.
 *          
 *          Variance Interpretation:
 *          The SV, SP, SH, and SM fields can be interpreted as "Squared Innovation Test
 *          Ratios" where:
 *          - Values < 1.0: Measurement was accepted by innovation gate
 *          - Values > 1.0: Measurement was rejected as inconsistent
 *          - Ratio = (innovation / max_allowed_innovation)²
 *          - Max allowed innovation determined by measurement uncertainty, prediction
 *            uncertainty, and gate parameters (EK2_MAG_I_GATE, EK2_HGT_I_GATE, etc.)
 *          
 *          Status Bitmasks:
 *          - faults: Filter fault status flags
 *          - timeouts: Measurement timeout bitmask
 *            * Bit 0: Position measurement timeout
 *            * Bit 1: Velocity measurement timeout
 *            * Bit 2: Height measurement timeout
 *            * Bit 3: Magnetometer measurement timeout
 *            * Bit 4: Airspeed measurement timeout
 *          - solution: Filter solution status (NavFilterStatusBit enum)
 *          - gps: GPS quality metrics and status
 *          - primary: Index of active primary EKF core (-1 if this core not primary)
 *          
 *          Units and Scaling:
 *          - sqrtvarV, sqrtvarP, sqrtvarH: cm (square root of variance)
 *          - sqrtvarM: milligauss
 *          - sqrtvarVT: cm/s
 *          - tiltErr: radians (float)
 *          - offsetNorth, offsetEast: meters (float) - position reset magnitude
 * 
 * @note Variance values naturally grow when measurements are unavailable and shrink
 *       when good measurements are fused. Sustained high variance may trigger failsafe.
 * 
 * @warning Filter status fields are critical for flight safety:
 *          - solution status must include "using GPS" for GPS-dependent modes
 *          - timeout flags indicate sensor failures requiring immediate attention
 *          - fault flags may trigger EKF failsafe and emergency landing
 *          - Position resets (offsetNorth/East) indicate filter discontinuities
 * 
 * @see NavFilterStatusBit enum for solution status bit definitions
 * @see NavEKF2_core::getVariances() for variance calculation
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:187-204
 */
// @LoggerMessage: NKF4
// @Description: EKF2 variances  SV, SP, SH and SM are probably best described as 'Squared Innovation Test Ratios' where values <1 tells us the measurement was accepted and >1 tells us it was rejected. They represent the square of the (innovation / maximum allowed innovation) where the innovation is the difference between predicted and measured value and the maximum allowed innovation is determined from the uncertainty of the measurement, uncertainty of the prediction and scaled using the number of standard deviations set by the innovation gate parameter for that measurement, eg EK2_MAG_I_GATE, EK2_HGT_I_GATE, etc
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: SV: Square root of the velocity variance
// @Field: SP: Square root of the position variance
// @Field: SH: Square root of the height variance
// @Field: SM: Magnetic field variance
// @Field: SVT: tilt error convergence metric
// @Field: errRP: Filtered error in roll/pitch estimate
// @Field: OFN: Most recent position reset (North component)
// @Field: OFE: Most recent position reset (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement)
// @Field: SS: Filter solution status
// @FieldBitmaskEnum: SS: NavFilterStatusBit
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index
struct PACKED log_NKF4 {
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


/**
 * @struct log_NKF5
 * @brief EKF2 optical flow, rangefinder, and terrain estimation data
 * 
 * @details This structure captures diagnostic data for optical flow sensors, rangefinders,
 *          and terrain height estimation. These sensors enable GPS-independent navigation
 *          and terrain-following flight modes.
 *          
 *          Optical Flow Data:
 *          - Flow innovations: Difference between measured and predicted optical flow
 *          - Normalized variance: Flow measurement consistency indicator
 *          - Separate innovations for navigation filter vs terrain estimator
 *          
 *          Terrain Estimation:
 *          - HAGL: Height Above Ground Level from terrain estimator
 *          - offset: Terrain height relative to EKF origin (NED down)
 *          - errHAGL: Uncertainty in terrain height estimate
 *          
 *          Rangefinder Data:
 *          - Range innovation: Measured vs predicted range
 *          - Measured range: Raw sensor reading
 *          
 *          Error Metrics:
 *          - angErr, velErr, posErr: Consolidated error magnitudes for filter health
 *          
 *          Coordinate Frame:
 *          - Flow innovations: Body frame (X = forward, Y = right)
 *          - Terrain offset: NED frame (positive down)
 *          
 *          Units and Scaling:
 *          - normInnov: dimensionless (0-255, normalized to uint8_t)
 *          - FIX, FIY: rad/s * 100 (optical flow rate innovations)
 *          - AFI: rad/s * 100 (auxiliary flow innovation)
 *          - HAGL: cm (centimeters above ground)
 *          - offset: cm (terrain offset relative to EKF origin)
 *          - RI: cm (rangefinder innovation)
 *          - meaRng: cm (measured range)
 *          - errHAGL: cm (terrain height uncertainty)
 *          - angErr, velErr, posErr: radians or m/s or m (float)
 * 
 * @note This message serves as a "general dumping ground" for miscellaneous EKF2
 *       diagnostic data that doesn't fit cleanly into other log structures.
 * 
 * @warning HAGL estimate is used for terrain-following modes and precision landing.
 *          Large errHAGL values or sustained rangefinder innovations indicate terrain
 *          estimation problems that may affect flight safety in terrain-following modes.
 * 
 * @see AP_OpticalFlow for optical flow sensor interface
 * @see AP_RangeFinder for rangefinder interface
 * @see NavEKF2_core::FuseOptFlow() for optical flow fusion
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:223-239
 */
// @LoggerMessage: NKF5
// @Description: EKF2 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
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
struct PACKED log_NKF5 {
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


/**
 * @struct log_NKQ
 * @brief EKF2 attitude quaternion representation
 * 
 * @details This structure logs the vehicle attitude as a quaternion, providing a
 *          singularity-free representation of 3D rotation. The quaternion defines
 *          the rotation from NED (North-East-Down) earth frame to the vehicle body
 *          frame (X-forward, Y-right, Z-down).
 *          
 *          Quaternion Convention:
 *          - ArduPilot uses Hamilton convention: q = q1 + q2*i + q3*j + q4*k
 *          - Rotation from NED to body frame
 *          - Unit quaternion: q1² + q2² + q3² + q4² = 1
 *          
 *          Advantages Over Euler Angles:
 *          - No gimbal lock singularities
 *          - More numerically stable for integration
 *          - Efficient for rotation composition
 *          - Better for high rotation rates or inverted flight
 *          
 *          Units:
 *          - All quaternion components: dimensionless (float, normalized to unit length)
 *          
 *          Conversion to Euler Angles:
 *          - Roll, pitch, yaw can be computed from quaternion if needed
 *          - NKF1 message provides Euler angle representation
 *          
 *          Usage:
 *          - Log replay and analysis requiring attitude reconstruction
 *          - Validation of Euler angle conversions in NKF1
 *          - Research and algorithm development
 * 
 * @note Quaternion logging is less intuitive than Euler angles for human interpretation
 *       but provides complete attitude information without ambiguity.
 * 
 * @see NavEKF2_core::calcQuat() for quaternion computation
 * @see NavEKF2_core::quat2angle() for quaternion to Euler conversion
 * @see log_NKF1 for Euler angle representation
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:250-258
 */
// @LoggerMessage: NKQ
// @Description: EKF2 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF2 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term
struct PACKED log_NKQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float q1;
    float q2;
    float q3;
    float q4;
};

/**
 * @struct log_NKT
 * @brief EKF2 timing diagnostics for performance monitoring and troubleshooting
 * 
 * @details This structure captures detailed timing information for NavEKF2 execution
 *          and IMU sample processing. Timing consistency is critical for accurate
 *          state estimation - irregular timing can introduce errors in integration
 *          and prediction steps.
 *          
 *          Timing Metrics:
 *          - IMU sample intervals: Time between IMU measurements (gyro + accel)
 *          - EKF update intervals: Time between EKF prediction/update cycles
 *          - Delta angle intervals: Time span for accumulated gyro measurements
 *          - Delta velocity intervals: Time span for accumulated accel measurements
 *          
 *          Min/Max Tracking:
 *          - Each metric tracks both minimum and maximum values over sampling period
 *          - Large spread (max - min) indicates timing jitter or scheduling issues
 *          - Consistent intervals indicate healthy real-time performance
 *          
 *          Units:
 *          - All timing fields: seconds (float)
 *          - timing_count: dimensionless count of samples averaged
 *          
 *          Expected Values:
 *          - IMU sample interval: ~1-10 ms depending on IMU configuration
 *          - EKF update interval: typically matches IMU sample rate
 *          - Delta angle/velocity intervals: should match IMU accumulation period
 *          
 *          Diagnostic Usage:
 *          - Identify scheduling problems causing irregular EKF updates
 *          - Detect IMU communication issues (large IMU interval jitter)
 *          - Validate IMU FIFO configuration and sample accumulation
 *          - Troubleshoot CPU overload affecting filter performance
 * 
 * @note This diagnostic is typically logged at lower rate than other EKF messages
 *       and represents statistics accumulated over multiple samples.
 * 
 * @warning Irregular timing can significantly degrade EKF performance:
 *          - Large IMU interval variations indicate IMU communication problems
 *          - Large EKF interval variations indicate CPU scheduling issues
 *          - Sustained timing irregularities may cause filter divergence
 * 
 * @see AP_InertialSensor for IMU sample timing
 * @see NavEKF2_core::readIMUData() for IMU data processing
 * @see NavEKF2_core::UpdateFilter() for EKF update timing
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:273-286
 */
// @LoggerMessage: NKT
// @Description: EKF2 timing information
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
struct PACKED log_NKT {
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

/**
 * @def LOG_STRUCTURE_FROM_NAVEKF2
 * @brief Registers all NavEKF2 log message definitions with the AP_Logger system
 * 
 * @details This macro defines the complete set of log message structure definitions
 *          for NavEKF2 telemetry. Each log structure is registered with AP_Logger
 *          using a format that specifies:
 *          
 *          Log Structure Format (per AP_Logger conventions):
 *          { message_id, sizeof(structure), "name", "format", "labels", "units", "multipliers", streaming }
 *          
 *          Format String Characters (field types):
 *          - Q: uint64_t (timestamp)
 *          - B: uint8_t (byte)
 *          - b: int8_t (signed byte)
 *          - H: uint16_t (unsigned short)
 *          - h: int16_t (signed short)
 *          - I: uint32_t (unsigned int)
 *          - i: int32_t (signed int)
 *          - f: float (32-bit floating point)
 *          - c: int16_t * 100 (centi-value, divide by 100)
 *          - C: uint16_t * 100 (unsigned centi-value)
 *          - e: int32_t * 100 (centi-value, divide by 100)
 *          
 *          Units String:
 *          - s: seconds
 *          - d: degrees
 *          - h: degrees (heading)
 *          - n: m/s (velocity)
 *          - m: meters (distance)
 *          - k: deg/s (angular rate)
 *          - G: gauss (magnetic field)
 *          - r: radians
 *          - -: dimensionless or not applicable
 *          
 *          Multipliers String:
 *          - F: First field (timestamp) multiplier
 *          - 0: Value used as-is (multiplier = 1)
 *          - B: Value * 0.01 (centi to base unit)
 *          - C: Value * 0.01 (centi to base unit)
 *          - ?: Not applicable or special handling
 *          
 *          Streaming Flag:
 *          - true: High-priority message for real-time streaming
 *          - false: Lower priority, may be dropped under bandwidth constraints
 *          
 *          Conditional Compilation:
 *          This macro is only defined when HAL_NAVEKF2_AVAILABLE is true, ensuring
 *          that log structures are not registered on builds where NavEKF2 is disabled
 *          (e.g., small boards with insufficient flash/RAM).
 *          
 *          Usage:
 *          This macro is expanded in the AP_Logger message registration array,
 *          typically in LogStructure.cpp files at the vehicle level. Ground control
 *          stations parse these definitions to interpret binary log files.
 * 
 * @note Format strings must exactly match the order and types of fields in the
 *       corresponding structure, or log parsing will fail.
 * 
 * @warning Changing message formats breaks compatibility with existing log files
 *          and ground station software. New message types or versions should be
 *          added rather than modifying existing structures.
 * 
 * @see AP_Logger::Log_Write() for log message writing
 * @see AP_Logger/LogStructure.h for log structure format documentation
 * @see Tools/Replay for log replay using these definitions
 * 
 * Source: libraries/AP_NavEKF2/LogStructure.h:288-307
 */
#if HAL_NAVEKF2_AVAILABLE
#define LOG_STRUCTURE_FROM_NAVEKF2        \
    { LOG_NKF0_MSG, sizeof(log_NKF0), \
      "NKF0","QBBccCCcccccccc","TimeUS,C,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD", "s#-m---mmmmmmmm", "F--B---BBBBBBBB" , true }, \
    { LOG_NKF1_MSG, sizeof(log_NKF1), \
      "NKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" , true }, \
    { LOG_NKF2_MSG, sizeof(log_NKF2), \
      "NKF2","QBbccccchhhhhhB","TimeUS,C,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI", "s#----nnGGGGGG-", "F-----BBCCCCCC-" , true }, \
    { LOG_NKF3_MSG, sizeof(log_NKF3), \
      "NKF3","QBcccccchhhccff","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT,RErr,ErSc", "s#nnnmmmGGGd?--", "F-BBBBBBCCCBB00" , true }, \
    { LOG_NKF4_MSG, sizeof(log_NKF4), \
      "NKF4","QBcccccfffHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------mm-----", "F-------??-----" , true }, \
    { LOG_NKF5_MSG, sizeof(log_NKF5), \
      "NKF5","QBBhhhcccCCfff","TimeUS,C,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos", "s#----m???mrnm", "F-----BBBBB000" , true }, \
    { LOG_NKQ_MSG, sizeof(log_NKQ), "NKQ", "QBffff", "TimeUS,C,Q1,Q2,Q3,Q4", "s#----", "F-0000" , true }, \
    { LOG_NKT_MSG, sizeof(log_NKT),   \
      "NKT", "QBIffffffff", "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax", "s#sssssssss", "F-000000000", true },
#else
#define LOG_STRUCTURE_FROM_NAVEKF2
#endif
