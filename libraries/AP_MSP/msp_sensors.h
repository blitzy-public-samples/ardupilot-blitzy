/**
 * @file msp_sensors.h
 * @brief MSP v2 sensor message structures for ArduPilot MSP protocol implementation
 * 
 * @details This file contains sensor data message structure definitions based on the
 *          MSP (MultiWii Serial Protocol) v2 sensor message specification. These
 *          structures define the wire format for sensor data exchanged between
 *          ArduPilot and MSP-compatible devices (OSD, flight controllers, etc.).
 *          
 *          All structures use PACKED attribute to ensure correct binary layout
 *          for the MSP v2 protocol wire format without padding.
 *          
 *          Protocol reference: src/main/msp/msp_protocol_v2_sensor_msg.h
 * 
 * @note These structures must maintain exact binary compatibility with MSP v2 protocol
 * @warning Do not modify structure layout or field sizes - breaks protocol compatibility
 */

#include <stdint.h>

#pragma once

/**
 * @namespace MSP
 * @brief MultiWii Serial Protocol (MSP) message structures and definitions
 * 
 * @details This namespace contains all MSP protocol-related message structures
 *          for sensor data communication. MSP is a lightweight binary protocol
 *          originally developed for MultiWii flight controllers, now widely
 *          supported by various flight controller firmware and peripherals.
 *          
 *          ArduPilot uses MSP primarily for:
 *          - OSD (On-Screen Display) integration
 *          - Telemetry with MSP-compatible devices
 *          - Sensor data exchange with external systems
 */
namespace MSP
{
// Protocol reference: src/main/msp/msp_protocol_v2_sensor_msg.h

/**
 * @struct msp_rangefinder_data_message_t
 * @brief MSP v2 rangefinder/distance sensor data message
 * 
 * @details Transmits distance measurements from rangefinder sensors (lidar, sonar, etc.)
 *          to MSP-compatible devices. Used for altitude hold, terrain following,
 *          and obstacle detection applications.
 * 
 * @note PACKED attribute ensures no padding between fields for wire protocol compatibility
 */
typedef struct PACKED {
    /**
     * @brief Measurement quality indicator
     * 
     * Quality of the distance measurement:
     * - 0 = Invalid/no reading
     * - 1-254 = Quality level (higher is better)
     * - 255 = Best possible quality/strong signal
     * 
     * Range: [0-255]
     */
    uint8_t quality;
    
    /**
     * @brief Distance measurement in millimeters
     * 
     * Measured distance from sensor to target:
     * - Positive values: Valid distance measurement in mm
     * - Negative values: Out of range (too far, too close, or no target detected)
     * - 0: May indicate sensor at minimum range or initialization
     * 
     * Units: millimeters [mm]
     * 
     * @note Negative values signal out-of-range conditions, not actual negative distances
     */
    int32_t distance_mm;
} msp_rangefinder_data_message_t;

/**
 * @struct msp_opflow_data_message_t
 * @brief MSP v2 optical flow sensor data message
 * 
 * @details Transmits optical flow measurements from image-based motion sensors.
 *          Optical flow sensors detect ground movement by analyzing consecutive
 *          camera frames, used for position hold and navigation when GPS unavailable.
 * 
 * @note PACKED attribute ensures no padding between fields for wire protocol compatibility
 */
typedef struct PACKED {
    /**
     * @brief Flow measurement quality indicator
     * 
     * Quality of optical flow measurement:
     * - 0 = Invalid/no flow detected (low light, featureless surface)
     * - 1-254 = Quality level (higher indicates better feature tracking)
     * - 255 = Best possible quality (strong features, good lighting)
     * 
     * Range: [0-255]
     */
    uint8_t quality;
    
    /**
     * @brief Optical flow motion in X axis (body frame forward/back)
     * 
     * Detected motion in sensor's X axis (typically vehicle forward direction).
     * Positive values indicate forward motion, negative indicate backward motion.
     * 
     * Units: sensor-specific units (typically milliradians or raw pixel flow)
     * 
     * @note Coordinate frame: body frame, X = forward direction
     * @note Scale factor depends on sensor height above ground and sensor characteristics
     */
    int32_t motion_x;
    
    /**
     * @brief Optical flow motion in Y axis (body frame right/left)
     * 
     * Detected motion in sensor's Y axis (typically vehicle right direction).
     * Positive values indicate rightward motion, negative indicate leftward motion.
     * 
     * Units: sensor-specific units (typically milliradians or raw pixel flow)
     * 
     * @note Coordinate frame: body frame, Y = right direction
     * @note Scale factor depends on sensor height above ground and sensor characteristics
     */
    int32_t motion_y;
} msp_opflow_data_message_t;

/**
 * @struct msp_gps_data_message_t
 * @brief MSP v2 GPS sensor data message with comprehensive positioning information
 * 
 * @details Transmits complete GPS/GNSS receiver data including position, velocity,
 *          accuracy estimates, and timing information. Supports multi-GPS configurations
 *          through instance field.
 *          
 *          This structure provides all data needed for navigation:
 *          - 3D position (latitude, longitude, altitude MSL)
 *          - 3D velocity in NED frame
 *          - Accuracy estimates for position and velocity
 *          - Satellite visibility and solution quality
 *          - UTC date/time
 * 
 * @note PACKED attribute ensures no padding between fields for wire protocol compatibility
 * @note All angles use decidegree (deg * 100) format for efficient integer transmission
 * @note Coordinate system: NED (North-East-Down) frame for velocities
 */
typedef struct PACKED {
    /**
     * @brief GPS sensor instance number
     * 
     * Identifies which GPS sensor in multi-GPS configurations (0 = primary, 1 = secondary, etc.)
     * Allows systems to use multiple GPS receivers for redundancy and accuracy improvement.
     */
    uint8_t  instance;
    
    /**
     * @brief GPS week number
     * 
     * GPS week number since GPS epoch (January 6, 1980).
     * Sentinel value 0xFFFF (65535) indicates GPS week not available or invalid.
     * 
     * Valid range: 0-65534, 0xFFFF = not available
     * 
     * @note GPS week resets every 1024 weeks (~19.7 years) - receivers must handle rollover
     */
    uint16_t gps_week;
    
    /**
     * @brief GPS time of week in milliseconds
     * 
     * Milliseconds since start of current GPS week (Sunday 00:00:00 UTC).
     * Provides high-precision timing within the week.
     * 
     * Units: milliseconds [ms]
     * Valid range: 0 to 604,800,000 (one week)
     */
    uint32_t ms_tow;
    
    /**
     * @brief GPS fix type/quality indicator
     * 
     * Indicates GPS solution quality:
     * - 0 = No fix
     * - 1 = Dead reckoning only
     * - 2 = 2D fix
     * - 3 = 3D fix
     * - 4 = GPS + dead reckoning
     * - 5 = Time-only fix
     * - 6 = RTK float
     * - 7 = RTK fixed
     * 
     * @note Higher values generally indicate better position accuracy
     */
    uint8_t  fix_type;
    
    /**
     * @brief Number of satellites visible/tracked
     * 
     * Count of satellites currently being used or tracked by GPS receiver.
     * More satellites generally improves accuracy and solution reliability.
     * 
     * Typical values: 4+ for 3D fix, 6+ for good accuracy
     */
    uint8_t  satellites_in_view;
    
    /**
     * @brief Horizontal position accuracy estimate
     * 
     * Estimated 1-sigma uncertainty of horizontal position (latitude/longitude).
     * Lower values indicate better position accuracy.
     * 
     * Units: centimeters [cm]
     * 
     * @note Actual error may exceed this estimate under poor conditions
     */
    uint16_t horizontal_pos_accuracy;
    
    /**
     * @brief Vertical position accuracy estimate
     * 
     * Estimated 1-sigma uncertainty of vertical position (altitude).
     * Typically 2-3x worse than horizontal accuracy.
     * 
     * Units: centimeters [cm]
     * 
     * @note Vertical accuracy depends heavily on satellite geometry (VDOP)
     */
    uint16_t vertical_pos_accuracy;
    
    /**
     * @brief Horizontal velocity accuracy estimate
     * 
     * Estimated 1-sigma uncertainty of horizontal velocity components (north/east).
     * 
     * Units: centimeters per second [cm/s]
     */
    uint16_t horizontal_vel_accuracy;
    
    /**
     * @brief Horizontal Dilution of Precision
     * 
     * Geometric quality factor indicating how satellite geometry affects
     * horizontal position accuracy. Lower values indicate better geometry.
     * 
     * Typical values:
     * - < 2: Excellent
     * - 2-5: Good
     * - 5-10: Moderate
     * - > 10: Poor
     * 
     * @note HDOP is dimensionless but often scaled by 100 in this field
     */
    uint16_t hdop;
    
    /**
     * @brief Longitude coordinate
     * 
     * East-West position in degrees scaled by 1e7 for integer transmission.
     * 
     * Format: degrees * 10,000,000
     * Range: -180° to +180° (-1,800,000,000 to +1,800,000,000)
     * - Positive: East of Prime Meridian
     * - Negative: West of Prime Meridian
     * 
     * Example: -1,227,890,000 = -122.789° (San Francisco)
     * 
     * @note Convert to decimal degrees: longitude_degrees = longitude / 10000000.0
     */
    int32_t  longitude;
    
    /**
     * @brief Latitude coordinate
     * 
     * North-South position in degrees scaled by 1e7 for integer transmission.
     * 
     * Format: degrees * 10,000,000
     * Range: -90° to +90° (-900,000,000 to +900,000,000)
     * - Positive: North of Equator
     * - Negative: South of Equator
     * 
     * Example: 376,890,000 = 37.689° (San Francisco)
     * 
     * @note Convert to decimal degrees: latitude_degrees = latitude / 10000000.0
     */
    int32_t  latitude;
    
    /**
     * @brief Altitude above Mean Sea Level (MSL)
     * 
     * Height above the WGS84 geoid (mean sea level reference).
     * Positive values are above MSL, negative values below MSL.
     * 
     * Units: centimeters [cm]
     * 
     * @note This is altitude relative to sea level, not ground level
     * @note Different from ellipsoid height - geoid separation applied
     */
    int32_t  msl_altitude;
    
    /**
     * @brief Velocity North component (NED frame)
     * 
     * Velocity in true North direction (not magnetic north).
     * Part of 3D velocity vector in NED (North-East-Down) coordinate frame.
     * 
     * Units: centimeters per second [cm/s]
     * - Positive: Moving North
     * - Negative: Moving South
     * 
     * @note NED frame: Earth-fixed frame with origin at current vehicle position
     */
    int32_t  ned_vel_north;
    
    /**
     * @brief Velocity East component (NED frame)
     * 
     * Velocity in true East direction.
     * Part of 3D velocity vector in NED (North-East-Down) coordinate frame.
     * 
     * Units: centimeters per second [cm/s]
     * - Positive: Moving East
     * - Negative: Moving West
     * 
     * @note NED frame: Earth-fixed frame with origin at current vehicle position
     */
    int32_t  ned_vel_east;
    
    /**
     * @brief Velocity Down component (NED frame)
     * 
     * Velocity in downward direction (toward Earth center).
     * Part of 3D velocity vector in NED (North-East-Down) coordinate frame.
     * 
     * Units: centimeters per second [cm/s]
     * - Positive: Moving Down (descending)
     * - Negative: Moving Up (climbing)
     * 
     * @note NED frame: Down is positive (opposite of altitude change)
     */
    int32_t  ned_vel_down;
    
    /**
     * @brief Ground course/track over ground
     * 
     * Direction of ground movement (track) relative to true North.
     * This is direction of actual movement, not vehicle heading.
     * 
     * Format: degrees * 100 (decidegrees)
     * Range: 0 to 36000 (0° to 360°)
     * - 0/36000 = North
     * - 9000 = East
     * - 18000 = South
     * - 27000 = West
     * 
     * Units: decidegrees [deg * 100]
     * 
     * @note This is track (direction of motion), not heading (direction vehicle points)
     * @note Only meaningful when vehicle is moving (velocity > threshold)
     */
    uint16_t ground_course;
    
    /**
     * @brief True heading/yaw angle
     * 
     * Vehicle heading relative to true North (requires dual GPS or compass).
     * 
     * Format: degrees * 100 (decidegrees)
     * Valid range: 0 to 36000 (0° to 360°)
     * Sentinel value: 65535 (0xFFFF) = no data available
     * - 0/36000 = North
     * - 9000 = East  
     * - 18000 = South
     * - 27000 = West
     * 
     * Units: decidegrees [deg * 100]
     * 
     * @note Requires dual GPS for moving baseline or compass for heading determination
     * @note Value 65535 indicates heading not available or not computed
     */
    uint16_t true_yaw;
    
    /**
     * @brief UTC year
     * 
     * Current year from GPS time converted to UTC.
     * 
     * Valid range: 1980-65534 (GPS epoch start to max uint16)
     * Example: 2024
     * 
     * @note GPS time does not include leap seconds; conversion to UTC may differ by ~18 seconds
     */
    uint16_t year;
    
    /**
     * @brief UTC month
     * 
     * Month of year from GPS time converted to UTC.
     * 
     * Valid range: 1-12 (January = 1, December = 12)
     */
    uint8_t  month;
    
    /**
     * @brief UTC day of month
     * 
     * Day of the month from GPS time converted to UTC.
     * 
     * Valid range: 1-31 (depending on month)
     */
    uint8_t  day;
    
    /**
     * @brief UTC hour
     * 
     * Hour of day in 24-hour format from GPS time converted to UTC.
     * 
     * Valid range: 0-23 (midnight = 0, 11 PM = 23)
     */
    uint8_t  hour;
    
    /**
     * @brief UTC minute
     * 
     * Minute of hour from GPS time converted to UTC.
     * 
     * Valid range: 0-59
     */
    uint8_t  min;
    
    /**
     * @brief UTC second
     * 
     * Second of minute from GPS time converted to UTC.
     * 
     * Valid range: 0-59 (60 during leap second insertion)
     * 
     * @note Does not include fractional seconds - use ms_tow for sub-second precision
     */
    uint8_t  sec;
} msp_gps_data_message_t;

/**
 * @struct msp_baro_data_message_t
 * @brief MSP v2 barometer/pressure sensor data message
 * 
 * @details Transmits barometric pressure and temperature measurements used for
 *          altitude estimation and temperature compensation. Barometers are the
 *          primary altitude sensor for most flight controllers.
 *          
 *          Multiple barometer instances supported for redundancy and voting.
 * 
 * @note PACKED attribute ensures no padding between fields for wire protocol compatibility
 */
typedef struct PACKED {
    /**
     * @brief Barometer sensor instance number
     * 
     * Identifies which barometer sensor in multi-sensor configurations.
     * 0 = primary, 1 = secondary, etc.
     */
    uint8_t instance;
    
    /**
     * @brief Measurement timestamp
     * 
     * System time in milliseconds when measurement was taken.
     * Used for sensor fusion and data synchronization.
     * 
     * Units: milliseconds [ms]
     * 
     * @note Timestamp should be from consistent time source (typically system uptime)
     */
    uint32_t time_ms;
    
    /**
     * @brief Atmospheric pressure measurement
     * 
     * Absolute atmospheric pressure reading from barometer sensor.
     * Used to calculate pressure altitude via standard atmosphere model.
     * 
     * Units: Pascals [Pa]
     * Typical range at sea level: ~101,325 Pa (1013.25 hPa)
     * Typical range at altitude: decreases ~12 Pa per meter altitude gain
     * 
     * @note Pressure decreases exponentially with altitude
     * @note Temperature compensation required for accurate altitude calculation
     */
    float pressure_pa;
    
    /**
     * @brief Temperature measurement
     * 
     * Ambient temperature from barometer's integrated temperature sensor.
     * Used for pressure sensor temperature compensation and air density calculation.
     * 
     * Format: temperature * 100 (centi-degrees)
     * Units: centi-degrees Celsius [°C * 100]
     * 
     * Example: 2550 = 25.50°C
     * 
     * @note Convert to Celsius: temp_celsius = temp / 100.0
     * @note Temperature affects pressure reading accuracy and altitude calculation
     */
    int16_t temp;
} msp_baro_data_message_t;

/**
 * @struct msp_compass_data_message_t
 * @brief MSP v2 magnetometer/compass sensor data message
 * 
 * @details Transmits 3-axis magnetic field measurements in vehicle body frame.
 *          Used for heading determination, compass calibration, and magnetic
 *          field anomaly detection.
 *          
 *          Magnetic field vector is measured in NED body frame:
 *          - X axis: Vehicle front (forward)
 *          - Y axis: Vehicle right
 *          - Z axis: Vehicle down
 *          
 *          Multiple compass instances supported for redundancy and interference mitigation.
 * 
 * @note PACKED attribute ensures no padding between fields for wire protocol compatibility
 * @note Coordinate frame: NED body frame (Front-Right-Down)
 */
typedef struct PACKED {
    /**
     * @brief Compass sensor instance number
     * 
     * Identifies which compass sensor in multi-sensor configurations.
     * 0 = primary, 1 = secondary, etc.
     * 
     * Typical configurations:
     * - Internal compass (on flight controller)
     * - External compass (on GPS module or standalone)
     */
    uint8_t instance;
    
    /**
     * @brief Measurement timestamp
     * 
     * System time in milliseconds when measurement was taken.
     * Used for sensor fusion, calibration, and data synchronization.
     * 
     * Units: milliseconds [ms]
     */
    uint32_t time_ms;
    
    /**
     * @brief Magnetic field X component (body frame forward/front)
     * 
     * Magnetic field strength along vehicle's forward (nose) axis.
     * Positive values point forward, negative point backward.
     * 
     * Units: milliGauss [mGauss]
     * 
     * Typical range: -6000 to +6000 mGauss (Earth's field ~250-650 mGauss total magnitude)
     * 
     * @note Body frame: X axis points toward vehicle front
     * @note Earth's magnetic field at mid-latitudes: ~500 mGauss total magnitude
     * @note Motor/ESC interference can add ±1000+ mGauss during flight
     */
    int16_t magX;
    
    /**
     * @brief Magnetic field Y component (body frame right)
     * 
     * Magnetic field strength along vehicle's right wing/side axis.
     * Positive values point right, negative point left.
     * 
     * Units: milliGauss [mGauss]
     * 
     * Typical range: -6000 to +6000 mGauss
     * 
     * @note Body frame: Y axis points toward vehicle right side
     * @note Calibration compensates for hard-iron and soft-iron interference
     */
    int16_t magY;
    
    /**
     * @brief Magnetic field Z component (body frame down)
     * 
     * Magnetic field strength along vehicle's down axis (toward Earth).
     * Positive values point down, negative point up.
     * 
     * Units: milliGauss [mGauss]
     * 
     * Typical range: -6000 to +6000 mGauss
     * 
     * @note Body frame: Z axis points downward (NED convention)
     * @note Vertical component varies with magnetic latitude (dip angle)
     * @note At magnetic equator: Z ≈ 0, at magnetic poles: Z ≈ ±600 mGauss
     */
    int16_t magZ;
} msp_compass_data_message_t;

/**
 * @struct msp_airspeed_data_message_t
 * @brief MSP v2 airspeed/differential pressure sensor data message
 * 
 * @details Transmits airspeed sensor measurements for fixed-wing aircraft.
 *          Airspeed sensors measure differential pressure between pitot and
 *          static ports to determine airspeed (relative wind velocity).
 *          
 *          Critical for fixed-wing flight:
 *          - Stall prevention (minimum airspeed monitoring)
 *          - Airspeed hold modes
 *          - TECS (Total Energy Control System) energy management
 *          - Wind estimation
 *          
 *          Multiple sensor instances supported for redundancy.
 * 
 * @note PACKED attribute ensures no padding between fields for wire protocol compatibility
 * @note Not typically used on multirotors (rely on GPS ground speed instead)
 */
typedef struct PACKED {
    /**
     * @brief Airspeed sensor instance number
     * 
     * Identifies which airspeed sensor in multi-sensor configurations.
     * 0 = primary, 1 = secondary, etc.
     * 
     * Multiple sensors improve reliability for safety-critical airspeed monitoring.
     */
    uint8_t instance;
    
    /**
     * @brief Measurement timestamp
     * 
     * System time in milliseconds when measurement was taken.
     * Used for sensor fusion, wind estimation, and data synchronization.
     * 
     * Units: milliseconds [ms]
     */
    uint32_t time_ms;
    
    /**
     * @brief Differential pressure measurement
     * 
     * Pressure difference between pitot (forward-facing) and static ports.
     * Used to calculate indicated airspeed via Bernoulli equation.
     * 
     * Units: sensor-specific differential pressure units (typically Pascals)
     * 
     * Typical range: 0-1000 Pa (0-45 m/s airspeed)
     * 
     * Airspeed calculation: velocity = sqrt(2 * pressure / air_density)
     * 
     * @note Raw differential pressure requires temperature compensation
     * @note Zero-offset calibration needed (pressure at zero airspeed)
     * @note Positive pressure indicates forward airflow through pitot tube
     */
    float pressure;
    
    /**
     * @brief Temperature measurement
     * 
     * Ambient temperature from airspeed sensor's integrated temperature sensor.
     * Used for air density calculation and sensor temperature compensation.
     * 
     * Format: temperature * 100 (centi-degrees)
     * Units: centi-degrees Celsius [°C * 100]
     * 
     * Example: 1530 = 15.30°C
     * 
     * @note Convert to Celsius: temp_celsius = temp / 100.0
     * @note Air density varies with temperature - affects airspeed calculation
     * @note Temperature compensation improves airspeed accuracy across flight envelope
     */
    int16_t temp;
} msp_airspeed_data_message_t;

/**
 * @note All MSP sensor message structures use PACKED attribute
 * 
 * The PACKED attribute (compiler-specific: __attribute__((packed)) for GCC)
 * ensures structures have no padding bytes between fields. This guarantees
 * binary layout matches MSP v2 wire protocol specification exactly.
 * 
 * Without PACKED, compiler may insert padding for alignment, causing:
 * - Incorrect structure size
 * - Field misalignment in transmitted data
 * - Protocol incompatibility with MSP devices
 * 
 * @warning Modifying field order, types, or sizes breaks MSP v2 protocol compatibility
 * @warning All MSP-compatible devices expect exact binary layout defined here
 */
}
