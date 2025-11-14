/**
 * @file definitions.h
 * @brief Centralized mathematical and physical constants for ArduPilot
 * 
 * @details This file provides a comprehensive collection of mathematical constants,
 *          physical constants, and unit conversion macros used throughout the
 *          ArduPilot autopilot system. It serves as the single source of truth
 *          for fundamental values to ensure consistency across all modules.
 *          
 *          Key categories:
 *          - Mathematical constants (π, golden ratio)
 *          - Angle conversion macros (degrees, radians, centidegrees)
 *          - Physical constants (gravity, Earth radius)
 *          - WGS84 geodetic constants for GPS calculations
 *          - Unit conversion macros (temperature, speed, distance, time)
 *          - Standard atmosphere parameters
 *          
 * @note All angle conversions follow right-hand rule conventions
 * @note Centidegrees (degrees × 100) are used for parameter storage to avoid
 *       floating-point precision issues in latitude/longitude representation
 * @warning Do not modify fundamental mathematical constants (M_PI, etc.) as
 *          external integrations and MAVLink protocols depend on these values
 * 
 * @see location.h for geographic calculations using these constants
 * @see ftype.h for floating-point precision abstraction
 * @see vector2.h, vector3.h, matrix3.h for mathematical operations using these constants
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <cmath>

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @brief Pi constant (π ≈ 3.141592653589793...)
 * 
 * @details High-precision definition of the mathematical constant pi,
 *          representing the ratio of a circle's circumference to its diameter.
 *          This definition provides extended precision beyond standard library
 *          implementations to ensure consistent behavior across all platforms
 *          and compilers.
 * 
 * @note Redefined here to ensure consistent precision across different platforms
 *       and standard library implementations. Some platforms define M_PI with
 *       lower precision or not at all.
 * @warning Do not modify this value - external integrations and coordinate
 *          transformations depend on this precise definition
 */
#ifdef M_PI
# undef M_PI
#endif
#define M_PI      (3.141592653589793238462643383279502884)

/**
 * @brief Pi divided by 2 (π/2 ≈ 1.5707963...)
 * 
 * @details Represents 90 degrees in radians. Commonly used in trigonometric
 *          calculations and coordinate frame transformations.
 * 
 * @note Calculated from M_PI to maintain consistency
 */
#ifdef M_PI_2
# undef M_PI_2
#endif
#define M_PI_2    (M_PI / 2)

/**
 * @brief Golden ratio (φ ≈ 1.6180339)
 * 
 * @details The golden ratio, approximately equal to (1 + √5) / 2.
 *          Used in some geometric and optimization calculations.
 * 
 * @note Stored as float for memory efficiency in embedded systems
 */
#define M_GOLDEN  1.6180339f

/**
 * @brief Two times pi (2π ≈ 6.2831853...)
 * 
 * @details Represents a full circle (360 degrees) in radians.
 *          Used frequently in rotational calculations and periodic functions.
 * 
 * @note Calculated from M_PI to maintain consistency
 */
#define M_2PI         (M_PI * 2)

/**
 * @brief Debug flag for bounds checking in mathematical objects
 * 
 * @details When enabled (set to 1), adds runtime bounds checking to vector
 *          and matrix index operations. This helps catch out-of-bounds access
 *          during development and debugging. Some objects (e.g., SoloGimbalEKF)
 *          include additional debug information when this flag is enabled.
 * 
 *          To enable: Define MATH_CHECK_INDEXES=1 in this file or in the
 *          top-level Makefile (e.g., Tools/Replay/Makefile for log replay).
 * 
 * @note Default: 0 (disabled) for production builds
 * @warning Enabling this flag adds runtime overhead to all vector/matrix
 *          operations and should only be used during development and debugging.
 *          Not suitable for flight on resource-constrained platforms.
 * 
 * @see vector2.h, vector3.h, matrix3.h for usage in mathematical objects
 */
#ifndef MATH_CHECK_INDEXES
  #define MATH_CHECK_INDEXES 0
#endif

/**
 * @brief Convert centidegrees to radians
 * 
 * @details Conversion factor from centidegrees (degrees × 100) to radians.
 *          Centidegrees are commonly used in ArduPilot for storing angular
 *          parameters as integers to avoid floating-point precision issues,
 *          particularly for latitude and longitude values.
 * 
 *          Formula: radians = centidegrees × (π / 18000)
 *                         = centidegrees × (π / 180 / 100)
 * 
 * @note 1 centidegree = 0.01 degrees = π/18000 radians
 * @note ArduPilot uses int32 centidegrees for parameter storage to maintain
 *       precision in latitude/longitude representation
 * 
 * @see RAD_TO_CDEG for inverse conversion
 * @see location.h for geographic coordinate usage
 */
#define CDEG_TO_RAD     (M_PI / 18000.0f)

/**
 * @brief Convert radians to centidegrees
 * 
 * @details Conversion factor from radians to centidegrees (degrees × 100).
 * 
 *          Formula: centidegrees = radians × (18000 / π)
 *                                = radians × (180 / π × 100)
 * 
 * @note 1 radian = 5729.577951... centidegrees
 * @note Result should be cast to integer type for parameter storage
 * 
 * @see CDEG_TO_RAD for inverse conversion
 */
#define RAD_TO_CDEG     (18000.0f / M_PI)

/**
 * @brief Convert degrees to radians
 * 
 * @details Standard conversion factor from degrees to radians.
 *          Used throughout ArduPilot for angle calculations in navigation,
 *          attitude control, and sensor processing.
 * 
 *          Formula: radians = degrees × (π / 180)
 * 
 * @note 1 degree = π/180 radians ≈ 0.0174533 radians
 * @note Positive angles follow right-hand rule convention
 * 
 * @see RAD_TO_DEG for inverse conversion
 * @see DEG_TO_RAD_DOUBLE for high-precision GPS calculations
 */
#define DEG_TO_RAD      (M_PI / 180.0f)

/**
 * @brief Convert radians to degrees
 * 
 * @details Standard conversion factor from radians to degrees.
 * 
 *          Formula: degrees = radians × (180 / π)
 * 
 * @note 1 radian = 180/π degrees ≈ 57.2957795 degrees
 * @note Positive angles follow right-hand rule convention
 * 
 * @see DEG_TO_RAD for inverse conversion
 * @see RAD_TO_DEG_DOUBLE for high-precision GPS calculations
 */
#define RAD_TO_DEG      (180.0f / M_PI)

/**
 * @brief High-precision double conversion: degrees to radians (GPS-specific)
 * 
 * @details Double-precision conversion factor for GPS coordinate calculations.
 *          This precision is critical when converting between LLH (Latitude,
 *          Longitude, Height) and ECEF (Earth-Centered Earth-Fixed) coordinate
 *          systems using WGS84 ellipsoid functions.
 * 
 *          Calculated as: asin(1) / 90 = π / 180 with double precision
 * 
 * @note Only available when ALLOW_DOUBLE_MATH_FUNCTIONS is defined
 * @note The precision here matters significantly for GPS accuracy - even small
 *       rounding errors can result in meter-level position errors
 * @warning Do not replace with float conversions in GPS coordinate transforms
 * 
 * @see RAD_TO_DEG_DOUBLE for inverse conversion
 * @see location.h for WGS84 coordinate transformation functions
 */
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
static const double DEG_TO_RAD_DOUBLE = asin(1) / 90;

/**
 * @brief High-precision double conversion: radians to degrees (GPS-specific)
 * 
 * @details Double-precision conversion factor for GPS coordinate calculations.
 * 
 *          Calculated as: 1 / DEG_TO_RAD_DOUBLE = 180 / π with double precision
 * 
 * @note Only available when ALLOW_DOUBLE_MATH_FUNCTIONS is defined
 * @note Maintains reciprocal relationship with DEG_TO_RAD_DOUBLE for consistency
 * 
 * @see DEG_TO_RAD_DOUBLE for inverse conversion
 */
static const double RAD_TO_DEG_DOUBLE = 1 / DEG_TO_RAD_DOUBLE;
#endif

/**
 * @brief Standard gravity acceleration (m/s²)
 * 
 * @details The standard acceleration due to gravity at Earth's surface,
 *          defined as exactly 9.80665 m/s². This is the internationally
 *          accepted standard value at sea level and 45° latitude.
 * 
 *          Used throughout ArduPilot for:
 *          - Accelerometer calibration and scaling
 *          - Inertial navigation calculations
 *          - Vertical velocity and altitude estimation
 *          - Aircraft performance calculations
 * 
 * @note This is a standard constant, not the actual local gravity which
 *       varies with latitude and altitude (9.78 to 9.83 m/s²)
 * @note Units: meters per second squared (m/s²)
 * 
 * @see AP_InertialSensor for accelerometer usage
 * @see AP_AHRS for inertial navigation
 */
#define GRAVITY_MSS     9.80665f

/**
 * @brief Earth radius in meters (WGS84 equatorial)
 * 
 * @details Equatorial radius of Earth according to the WGS84 ellipsoid model,
 *          used for great-circle distance calculations and coordinate conversions.
 *          
 *          Value: 6,378,100 meters (≈ 6,378.1 km)
 * 
 * @note This is the WGS84 equatorial radius, simplified for computational efficiency
 * @note For more precise geodetic calculations, use WGS84_A (6,378,137 m)
 * @note Earth is not a perfect sphere - polar radius is about 21 km smaller
 * 
 * @see WGS84_A for precise equatorial radius
 * @see location.h for great-circle distance calculations
 * @see LATLON_TO_M for latitude/longitude to meter conversions
 */
#define RADIUS_OF_EARTH 6378100

/**
 * @brief Convert latitude/longitude degrees to meters
 * 
 * @details Conversion factor from degrees of latitude (or longitude at equator)
 *          to meters. Calculated as: (Earth circumference / 360) / 1000
 *          = (2π × RADIUS_OF_EARTH / 360) / 1000 ≈ 0.011131884502145034
 * 
 * @note For longitude, this value must be scaled by cos(latitude) as longitude
 *       lines converge toward the poles
 * @note For latitude, this conversion is approximately constant (varies < 1%
 *       due to Earth's ellipsoid shape)
 * @note Units: input in degrees, output in meters
 * 
 * @warning This does NOT include longitude scaling which is location-dependent.
 *          Multiply by cos(latitude) for longitude-to-meter conversions.
 * 
 * @see LATLON_TO_M_INV for inverse conversion
 * @see LATLON_TO_CM for centimeter conversion
 * @see location.h for location-aware distance calculations
 */
#define LATLON_TO_M     0.011131884502145034

/**
 * @brief Convert meters to latitude/longitude degrees (inverse)
 * 
 * @details Inverse conversion factor from meters to degrees of latitude
 *          (or longitude at equator). Value: 1 / LATLON_TO_M ≈ 89.83204953368922
 * 
 * @note For longitude, result must be scaled by 1/cos(latitude)
 * @note Units: input in meters, output in degrees
 * 
 * @see LATLON_TO_M for forward conversion
 */
#define LATLON_TO_M_INV 89.83204953368922

/**
 * @brief Convert latitude/longitude degrees to centimeters
 * 
 * @details Conversion factor from degrees to centimeters.
 *          Value: LATLON_TO_M × 100 ≈ 1.1131884502145034
 * 
 * @note Subject to same longitude scaling requirements as LATLON_TO_M
 * @note Units: input in degrees, output in centimeters
 * 
 * @see LATLON_TO_M for meter conversion
 */
#define LATLON_TO_CM    1.1131884502145034

/**
 * @brief WGS84 semi-major axis (equatorial radius) in meters
 * 
 * @details The semi-major axis (equatorial radius) of the WGS84 reference
 *          ellipsoid used by GPS and most modern navigation systems.
 *          This is the most precise value for Earth's equatorial radius.
 * 
 *          Value: 6,378,137.0 meters (exactly, by WGS84 definition)
 * 
 * @note WGS84 (World Geodetic System 1984) is the standard coordinate system
 *       used by GPS and international aviation
 * @note This is more precise than RADIUS_OF_EARTH (6,378,100 m) used for
 *       simplified calculations
 * 
 * @see WGS84_B for semi-minor axis (polar radius)
 * @see location.h for WGS84-based coordinate transformations
 */
static const double WGS84_A = 6378137.0;

/**
 * @brief WGS84 inverse flattening of the Earth
 * 
 * @details The inverse of the flattening factor (1/f) for the WGS84 ellipsoid.
 *          Flattening describes how much Earth deviates from a perfect sphere,
 *          being slightly oblate (flattened at the poles).
 * 
 *          Value: 298.257223563 (exactly, by WGS84 definition)
 *          
 *          Flattening f = (a - b) / a, where:
 *          - a = semi-major axis (equatorial radius)
 *          - b = semi-minor axis (polar radius)
 * 
 * @note Inverse flattening is used instead of flattening directly to avoid
 *       representing very small numbers (f ≈ 0.00335)
 * 
 * @see WGS84_F for flattening factor
 */
static const double WGS84_IF = 298.257223563;

/**
 * @brief WGS84 flattening of the Earth
 * 
 * @details The flattening factor (f) for the WGS84 ellipsoid, calculated as
 *          1 / WGS84_IF ≈ 0.00335281066474748.
 *          
 *          This represents how much Earth's shape deviates from a perfect sphere,
 *          with the polar radius being about 21.4 km less than the equatorial radius.
 * 
 * @note Calculated from WGS84_IF to maintain consistency
 * @see WGS84_IF for inverse flattening
 * @see WGS84_B for calculated semi-minor axis
 */
static const double WGS84_F = ((double)1.0 / WGS84_IF);

/**
 * @brief WGS84 semi-minor axis (polar radius) in meters
 * 
 * @details The semi-minor axis (polar radius) of the WGS84 reference ellipsoid,
 *          calculated from the semi-major axis and flattening:
 *          b = a × (1 - f) ≈ 6,356,752.314245 meters
 * 
 * @note This is approximately 21.385 km less than the equatorial radius
 * @note Calculated from WGS84_A and WGS84_F to maintain consistency
 * 
 * @see WGS84_A for semi-major axis (equatorial radius)
 */
static const double WGS84_B = (WGS84_A * (1 - WGS84_F));

/**
 * @brief WGS84 first eccentricity of the Earth ellipsoid
 * 
 * @details The first eccentricity (e) of the WGS84 ellipsoid, which describes
 *          how much the ellipse deviates from a circle. Calculated as:
 *          e = √(2f - f²) ≈ 0.0818191908426215
 * 
 *          Used in geodetic coordinate transformations between LLH (Latitude,
 *          Longitude, Height) and ECEF (Earth-Centered Earth-Fixed) coordinates.
 * 
 * @note Only available when ALLOW_DOUBLE_MATH_FUNCTIONS is defined (requires sqrt)
 * @note First eccentricity is used more commonly than second eccentricity in
 *       geodetic calculations
 * 
 * @see location.h for coordinate transformation functions using eccentricity
 * @see DEG_TO_RAD_DOUBLE for high-precision angle conversions used with WGS84
 */
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
static const double WGS84_E = (sqrt(2 * WGS84_F - WGS84_F * WGS84_F));
#endif

/**
 * @brief Convert Celsius to Kelvin
 * 
 * @details Converts temperature from degrees Celsius to Kelvin.
 *          Formula: K = °C + 273.15
 * 
 * @param temp Temperature in degrees Celsius
 * @return Temperature in Kelvin
 * 
 * @note Absolute zero is 0 K = -273.15 °C
 * @note Used in thermodynamic calculations and atmospheric models
 */
#define C_TO_KELVIN(temp) (temp + 273.15f)

/**
 * @brief Convert Kelvin to Celsius
 * 
 * @details Converts temperature from Kelvin to degrees Celsius.
 *          Formula: °C = K - 273.15
 * 
 * @param temp Temperature in Kelvin
 * @return Temperature in degrees Celsius
 */
#define KELVIN_TO_C(temp) (temp - 273.15f)

/**
 * @brief Convert Fahrenheit to Celsius
 * 
 * @details Converts temperature from degrees Fahrenheit to degrees Celsius.
 *          Formula: °C = (°F - 32) × 5/9
 * 
 * @param temp Temperature in degrees Fahrenheit
 * @return Temperature in degrees Celsius
 * 
 * @note Water freezes at 32°F = 0°C, boils at 212°F = 100°C
 */
#define F_TO_C(temp) ((temp - 32) * 5/9)

/**
 * @brief Convert Fahrenheit to Kelvin
 * 
 * @details Converts temperature from degrees Fahrenheit to Kelvin.
 *          Formula: K = (°F - 32) × 5/9 + 273.15
 * 
 * @param temp Temperature in degrees Fahrenheit
 * @return Temperature in Kelvin
 * 
 * @note Implemented as composition of F_TO_C and C_TO_KELVIN
 */
#define F_TO_KELVIN(temp) C_TO_KELVIN(F_TO_C(temp))

/**
 * @brief Convert Celsius to Fahrenheit
 * 
 * @details Converts temperature from degrees Celsius to degrees Fahrenheit.
 *          Formula: °F = °C × 9/5 + 32
 * 
 * @param temp Temperature in degrees Celsius
 * @return Temperature in degrees Fahrenheit
 */
#define C_TO_F(temp) ((temp * 9/5) + 32)

/**
 * @brief Convert meters per second to knots
 * 
 * @details Converts velocity from meters per second to nautical miles per hour (knots).
 *          Formula: knots = m/s × 1.94384449
 *          
 *          1 knot = 1 nautical mile per hour = 1.852 km/h = 0.51444... m/s
 * 
 * @note Knots are the standard unit for airspeed and wind speed in aviation
 * @note 1 nautical mile = 1,852 meters (by international definition)
 * 
 * @see KNOTS_TO_M_PER_SEC for inverse conversion
 */
#define M_PER_SEC_TO_KNOTS 1.94384449f

/**
 * @brief Convert knots to meters per second
 * 
 * @details Converts velocity from nautical miles per hour (knots) to meters per second.
 *          Formula: m/s = knots / 1.94384449 ≈ knots × 0.51444
 * 
 * @note Calculated as reciprocal of M_PER_SEC_TO_KNOTS for consistency
 * 
 * @see M_PER_SEC_TO_KNOTS for forward conversion
 * @see KNOTS_TO_METERS_PER_SECOND for alternative definition
 */
#define KNOTS_TO_M_PER_SEC (1/M_PER_SEC_TO_KNOTS)

/**
 * @brief Convert kilometers per hour to meters per second
 * 
 * @details Converts velocity from km/h to m/s.
 *          Formula: m/s = km/h × 0.27777778 = km/h / 3.6
 * 
 * @note Common conversion for ground vehicle speeds
 * @note 1 km/h = 1000 m / 3600 s = 5/18 m/s ≈ 0.27777778 m/s
 */
#define KM_PER_HOUR_TO_M_PER_SEC 0.27777778f

/**
 * @brief ISA specific gas constant for dry air (J/(kg·K))
 * 
 * @details The specific gas constant for dry air used in International Standard
 *          Atmosphere (ISA) calculations. Value: 287.26 J/(kg·K)
 *          
 *          Used in the ideal gas law: P = ρ × R × T, where:
 *          - P = pressure (Pa)
 *          - ρ = density (kg/m³)
 *          - R = specific gas constant (J/(kg·K))
 *          - T = temperature (K)
 * 
 * @note Reference: "Aerodynamics for Engineering Students, Third Edition"
 *       by E.L. Houghton and N.B. Carruthers
 * @note Units: Joules per kilogram-Kelvin (J/(kg·K))
 * 
 * @see SSL_AIR_PRESSURE, SSL_AIR_DENSITY, SSL_AIR_TEMPERATURE
 * @see AP_Baro for barometric altitude calculations
 */
#define ISA_GAS_CONSTANT 287.26f

/**
 * @brief ISA temperature lapse rate (K/m)
 * 
 * @details The rate at which temperature decreases with altitude in the
 *          troposphere according to the International Standard Atmosphere model.
 *          Value: 0.0065 K/m = 6.5 K/km
 *          
 *          Temperature decreases approximately 6.5°C per 1000 meters of altitude
 *          gain up to the tropopause (approximately 11 km).
 * 
 * @note Valid in troposphere only (sea level to ~11 km altitude)
 * @note Units: Kelvin per meter (K/m)
 * @note Used for altitude estimation from temperature and pressure
 * 
 * @see AP_Baro for barometric altitude calculations using lapse rate
 */
#define ISA_LAPSE_RATE 0.0065f

/**
 * @brief Standard sea level air density (kg/m³)
 * 
 * @details Air density at standard sea level conditions according to
 *          International Standard Atmosphere (ISA).
 *          Value: 1.225 kg/m³
 * 
 * @note Standard conditions: 15°C (288.15 K), 101325 Pa, 0% humidity
 * @note Units: kilograms per cubic meter (kg/m³)
 * @note Reference: https://en.wikipedia.org/wiki/Standard_sea_level
 * 
 * @see SSL_AIR_PRESSURE, SSL_AIR_TEMPERATURE
 * @see AP_Airspeed for true airspeed calculations
 */
#define SSL_AIR_DENSITY         1.225f // kg/m^3

/**
 * @brief Standard sea level air pressure (Pa)
 * 
 * @details Atmospheric pressure at standard sea level conditions according to
 *          International Standard Atmosphere (ISA).
 *          Value: 101325.01576 Pa = 1013.25 hPa = 29.92 inHg
 * 
 * @note Standard conditions: 15°C (288.15 K), 0% humidity
 * @note Units: Pascals (Pa)
 * @note 1 hPa (hectopascal) = 100 Pa = 1 millibar
 * @note Reference: https://en.wikipedia.org/wiki/Standard_sea_level
 * 
 * @see SSL_AIR_DENSITY, SSL_AIR_TEMPERATURE
 * @see AP_Baro for pressure altitude calculations
 */
#define SSL_AIR_PRESSURE 101325.01576f // Pascal

/**
 * @brief Standard sea level air temperature (K)
 * 
 * @details Temperature at standard sea level conditions according to
 *          International Standard Atmosphere (ISA).
 *          Value: 288.15 K = 15°C = 59°F
 * 
 * @note Standard conditions: 101325 Pa pressure, 0% humidity
 * @note Units: Kelvin (K)
 * @note Reference: https://en.wikipedia.org/wiki/Standard_sea_level
 * 
 * @see SSL_AIR_DENSITY, SSL_AIR_PRESSURE
 * @see C_TO_KELVIN, KELVIN_TO_C for temperature conversions
 */
#define SSL_AIR_TEMPERATURE    288.15f // K

/**
 * @brief Convert inches of water to Pascals
 * 
 * @details Converts pressure from inches of water column (inH₂O) to Pascals.
 *          Value: 1 inH₂O ≈ 248.84 Pa
 *          
 *          Used for differential pressure measurements, particularly in
 *          airspeed sensors and pitot tubes.
 * 
 * @note Common unit for measuring small pressure differences in aviation
 * @note Units: input in inH₂O, output in Pascals (Pa)
 * 
 * @see AP_Airspeed for airspeed sensor pressure measurements
 */
#define INCH_OF_H2O_TO_PASCAL 248.84f

/**
 * @brief Convert microtesla to milligauss
 * 
 * @details Converts magnetic field strength from microtesla (µT) to milligauss (mGauss).
 *          Formula: mGauss = µT × 10
 *          
 *          1 Tesla = 10,000 Gauss
 *          1 µT = 0.01 Gauss = 10 mGauss
 * 
 * @note Magnetometers often report in µT or mGauss
 * @note Earth's magnetic field is approximately 25-65 µT (250-650 mGauss)
 * @note Units: input in microtesla (µT), output in milligauss (mGauss)
 * 
 * @see AP_Compass for magnetometer measurements
 * @see NTESLA_TO_MGAUSS for nanotesla conversion
 */
#define UTESLA_TO_MGAUSS   10.0f // uT to mGauss conversion

/**
 * @brief Convert nanotesla to milligauss
 * 
 * @details Converts magnetic field strength from nanotesla (nT) to milligauss (mGauss).
 *          Formula: mGauss = nT × 0.01
 *          
 *          1 µT = 1000 nT
 *          1 nT = 0.01 mGauss
 * 
 * @note Used for high-resolution magnetometer measurements
 * @note Units: input in nanotesla (nT), output in milligauss (mGauss)
 * 
 * @see AP_Compass for magnetometer measurements
 * @see UTESLA_TO_MGAUSS for microtesla conversion
 */
#define NTESLA_TO_MGAUSS   0.01f // nT to mGauss conversion

/**
 * @brief Nanoseconds per second
 * 
 * @details Number of nanoseconds in one second: 1,000,000,000 ns = 1 s
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers (e.g., NuttX clock.h)
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * 
 * @see AP_NSEC_PER_USEC, AP_USEC_PER_SEC
 */
#define AP_NSEC_PER_SEC   1000000000ULL

/**
 * @brief Nanoseconds per microsecond
 * 
 * @details Number of nanoseconds in one microsecond: 1,000 ns = 1 µs
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * 
 * @see AP_NSEC_PER_SEC, AP_USEC_PER_MSEC
 */
#define AP_NSEC_PER_USEC  1000ULL

/**
 * @brief Microseconds per second
 * 
 * @details Number of microseconds in one second: 1,000,000 µs = 1 s
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * @note Commonly used for timing measurements in ArduPilot
 * 
 * @see AP_HAL::Scheduler for timing functions
 */
#define AP_USEC_PER_SEC   1000000ULL

/**
 * @brief Microseconds per millisecond
 * 
 * @details Number of microseconds in one millisecond: 1,000 µs = 1 ms
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * 
 * @see AP_USEC_PER_SEC, AP_MSEC_PER_SEC
 */
#define AP_USEC_PER_MSEC  1000ULL

/**
 * @brief Milliseconds per second
 * 
 * @details Number of milliseconds in one second: 1,000 ms = 1 s
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * @note Commonly used for scheduler timing and timeouts
 * 
 * @see AP_HAL::Scheduler::millis()
 */
#define AP_MSEC_PER_SEC   1000ULL

/**
 * @brief Seconds per hour
 * 
 * @details Number of seconds in one hour: 3,600 s = 1 hour
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * 
 * @see AP_MSEC_PER_HOUR, AP_SEC_PER_WEEK
 */
#define AP_SEC_PER_HOUR   (3600ULL)

/**
 * @brief Milliseconds per hour
 * 
 * @details Number of milliseconds in one hour: 3,600,000 ms = 1 hour
 *          Calculated as: AP_SEC_PER_HOUR × AP_MSEC_PER_SEC
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * 
 * @see AP_SEC_PER_HOUR, AP_MSEC_PER_SEC
 */
#define AP_MSEC_PER_HOUR  (AP_SEC_PER_HOUR * AP_MSEC_PER_SEC)

/**
 * @brief Seconds per week
 * 
 * @details Number of seconds in one week: 604,800 s = 7 days = 1 week
 *          Calculated as: 7 × 86,400 (seconds per day)
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * @note Used for GPS week calculations
 * 
 * @see AP_GPS for GPS time-of-week (TOW) calculations
 */
#define AP_SEC_PER_WEEK   (7ULL * 86400ULL)

/**
 * @brief Milliseconds per week
 * 
 * @details Number of milliseconds in one week: 604,800,000 ms = 1 week
 *          Calculated as: AP_SEC_PER_WEEK × AP_MSEC_PER_SEC
 * 
 * @note AP_ prefix used to prevent conflicts with OS headers
 * @note Type: unsigned long long (64-bit) to prevent overflow
 * @note Used for GPS week rollover calculations
 * 
 * @see AP_GPS for GPS time handling
 */
#define AP_MSEC_PER_WEEK  (AP_SEC_PER_WEEK * AP_MSEC_PER_SEC)

/**
 * @brief Convert knots to meters per second
 * 
 * @details Alternative definition for knots to m/s conversion.
 *          Value: 0.51444... m/s per knot
 *          
 *          1 knot = 1 nautical mile/hour = 1.852 km/h = 0.51444... m/s
 * 
 * @note This is an alternative to KNOTS_TO_M_PER_SEC with slightly different
 *       precision in the constant representation
 * @note Units: input in knots, output in meters per second (m/s)
 * 
 * @see KNOTS_TO_M_PER_SEC for reciprocal-based definition
 * @see M_PER_SEC_TO_KNOTS for inverse conversion
 */
#define KNOTS_TO_METERS_PER_SECOND 0.51444

/**
 * @brief Convert feet to meters
 * 
 * @details Converts distance from feet to meters.
 *          Value: 1 foot = 0.3048 meters (exactly, by international definition)
 * 
 * @note International foot definition (since 1959)
 * @note Units: input in feet (ft), output in meters (m)
 * @note Commonly used for altitude conversions in aviation (US uses feet)
 * 
 * @see METRES_TO_FEET for inverse conversion
 * @see AP_Baro for altitude measurements
 */
#define FEET_TO_METERS 0.3048

/**
 * @brief Convert meters to feet
 * 
 * @details Converts distance from meters to feet.
 *          Value: 1 meter = 3.280839895... feet
 *          Formula: feet = meters / 0.3048
 * 
 * @note High precision provided for accurate altitude conversions
 * @note Units: input in meters (m), output in feet (ft)
 * 
 * @see FEET_TO_METERS for inverse conversion
 */
#define METRES_TO_FEET 3.280839895013123

/**
 * @brief Convert amp-milliseconds to milliamp-hours
 * 
 * @details Converts electrical charge from amp-milliseconds (A·ms) to
 *          milliamp-hours (mAh), the standard unit for battery capacity.
 *          
 *          Formula: mAh = A·ms × 0.000277777778
 *                       = A·ms × (1/1000) × (1/3600) × 1000
 *                       = A·ms / 3600
 *          
 *          Derivation:
 *          - ms → s: divide by 1000
 *          - s → hours: divide by 3600
 *          - A → mA: multiply by 1000
 *          - Combined: (1/1000) × (1/3600) × 1000 = 1/3600
 * 
 * @note Used for integrating current measurements to estimate battery consumption
 * @note Units: input in amp-milliseconds (A·ms), output in milliamp-hours (mAh)
 * 
 * @see AP_BattMonitor for battery capacity calculations
 * @see AUS_TO_MAH for microsecond-based conversion
 */
#define AMS_TO_MAH 0.000277777778f

/**
 * @brief Convert amp-microseconds to milliamp-hours
 * 
 * @details Converts electrical charge from amp-microseconds (A·µs) to
 *          milliamp-hours (mAh).
 *          
 *          Formula: mAh = A·µs × 0.0000002778
 *                       = A·µs / 3,600,000,000
 * 
 * @note Used for high-resolution current integration
 * @note Units: input in amp-microseconds (A·µs), output in milliamp-hours (mAh)
 * 
 * @see AP_BattMonitor for battery capacity calculations
 * @see AMS_TO_MAH for millisecond-based conversion
 */
#define AUS_TO_MAH 0.0000002778f

/**
 * @brief Convert density from kg/m³ to g/cm³
 * 
 * @details Converts density from kilograms per cubic meter to grams per cubic centimeter.
 *          Formula: g/cm³ = kg/m³ × 0.001
 *          
 *          Since 1 kg = 1000 g and 1 m³ = 1,000,000 cm³:
 *          kg/m³ = (1000 g) / (1,000,000 cm³) = 0.001 g/cm³
 * 
 * @param x Density in kg/m³
 * @return Density in g/cm³
 * 
 * @note 1 g/cm³ = 1000 kg/m³ (e.g., water density)
 * @note Units: input in kg/m³, output in g/cm³
 * 
 * @see SSL_AIR_DENSITY for standard air density in kg/m³
 */
#define KG_PER_M3_TO_G_PER_CM3(x) (0.001 * x)
