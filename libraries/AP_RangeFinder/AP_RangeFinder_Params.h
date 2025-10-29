/**
 * @file AP_RangeFinder_Params.h
 * @brief Per-instance rangefinder parameter storage using AP_Param system
 * 
 * This file defines the parameter group for each rangefinder instance (up to 10 supported).
 * Parameters are persisted to EEPROM via AP_Param and enable configuration of sensor type,
 * mounting position, operating range limits, calibration factors, and power management.
 * 
 * @note Each rangefinder instance has its own AP_RangeFinder_Params object with separate
 *       parameter storage, allowing mixed sensor types and configurations
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/**
 * @class AP_RangeFinder_Params
 * @brief Parameter storage for a single rangefinder instance
 * 
 * @details This class holds all configurable parameters for one rangefinder sensor.
 *          ArduPilot supports up to 10 rangefinder instances, each with its own parameter set.
 *          Parameters control sensor type selection, calibration, mounting orientation,
 *          operating range limits, and power-saving features.
 *          
 *          The class integrates with AP_Param system for:
 *          - EEPROM persistence across reboots
 *          - Ground station parameter access via MAVLink
 *          - Parameter versioning and migration
 *          
 *          Parameter changes typically require reboot to reinitialize sensors with new configuration.
 * 
 * @note This class uses CLASS_NO_COPY to prevent parameter object copying since each
 *       instance is tied to a specific rangefinder backend
 * 
 * @warning Incorrect parameter configuration can result in invalid distance readings,
 *          affecting altitude hold, terrain following, and obstacle avoidance
 */
class AP_RangeFinder_Params {
public:
    /**
     * @brief AP_Param GroupInfo array for parameter registration and EEPROM persistence
     * 
     * @details Static table defining parameter metadata for AP_Param system:
     *          - Parameter names (RNGFND1_TYPE, RNGFND1_PIN, etc.)
     *          - Storage offsets within class
     *          - Default values
     *          - Parameter type information
     *          
     *          Defined in AP_RangeFinder_Params.cpp and used by AP_Param for:
     *          - Ground station parameter discovery
     *          - EEPROM save/load operations
     *          - Parameter validation and conversion
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Constructor for AP_RangeFinder_Params
     * 
     * @details Initializes parameter object with default values defined in var_info table.
     *          AP_Param system handles actual value loading from EEPROM after construction.
     *          
     *          Default initialization occurs before EEPROM values are applied, providing
     *          fallback configuration if parameters are unset or corrupted.
     */
    AP_RangeFinder_Params(void);
    
    /**
     * @brief Convert legacy _MIN/_MAX parameters to min_distance/max_distance
     * 
     * @details Migrates parameters from older firmware versions that used different naming:
     *          - Old: RNGFND1_MIN_CM, RNGFND1_MAX_CM (integer centimeters)
     *          - New: RNGFND1_MIN_DISTANCE, RNGFND1_MAX_DISTANCE (float meters)
     *          
     *          Called during initialization to ensure backwards compatibility when upgrading
     *          from older ArduPilot versions. Only converts if old parameters exist and
     *          new parameters are at default values.
     * 
     * @note This method ensures seamless firmware upgrades without requiring users to
     *       reconfigure their rangefinder parameters
     */
    void convert_min_max_params();

    /**
     * @brief Prevent parameter object copying
     * 
     * @details CLASS_NO_COPY macro deletes copy constructor and assignment operator.
     *          Parameter objects are tied to specific rangefinder instances and should
     *          never be copied. Copying would duplicate EEPROM storage references and
     *          cause parameter corruption.
     */
    CLASS_NO_COPY(AP_RangeFinder_Params);

    /**
     * @brief Sensor position offset in body frame (meters)
     * 
     * @details 3D position vector from vehicle center-of-gravity to sensor:
     *          - X: Forward (positive forward, negative backward)
     *          - Y: Right (positive right, negative left)
     *          - Z: Down (positive down, negative up)
     *          
     *          Body frame follows NED convention (North-East-Down).
     *          Accurate position offsets enable multi-rangefinder configurations
     *          where sensors are mounted at different locations for obstacle avoidance.
     * 
     * @note Critical for precision landing and obstacle detection - incorrect offsets
     *       can cause position estimation errors affecting automatic flight modes
     */
    AP_Vector3f pos_offset;
    
    /**
     * @brief Calibration scaling factor applied to sensor output (dimensionless multiplier)
     * 
     * @details Multiplier applied to raw sensor reading for calibration:
     *          calibrated_distance = (raw_reading * scaling) + offset
     *          
     *          Used primarily for analog rangefinders to linearize voltage-to-distance
     *          transfer function. Default is 1.0 (no scaling).
     * 
     * @note Analog sensors may have non-linear response requiring scaling adjustment
     *       to match actual measured distances
     */
    AP_Float scaling;
    
    /**
     * @brief Zero offset correction added to scaled reading (meters)
     * 
     * @details Offset added after scaling for zero-point calibration:
     *          calibrated_distance = (raw_reading * scaling) + offset
     *          
     *          Compensates for sensor bias or mounting effects. Default is 0.0 meters.
     * 
     * @note For analog rangefinders, offset linearizes the voltage-to-distance transfer
     *       function along with scaling parameter
     */
    AP_Float offset;
    
    /**
     * @brief Altitude above ground for automatic sensor power-down (centimeters)
     * 
     * @details When estimated terrain height exceeds this value, sensor is powered down
     *          to save energy. Set to 0 to disable power saving. Typical value: 1000 cm (10m)
     *          for short-range sensors that provide no useful data at high altitudes.
     *          
     *          Uses set_estimated_terrain_height() from frontend for altitude estimation.
     * 
     * @note Reduces power consumption during high-altitude flight where ground distance
     *       exceeds sensor maximum range
     */
    AP_Int16 powersave_range;
    
    /**
     * @brief Minimum valid measurement range (meters)
     * 
     * @details Distance readings below this threshold are rejected as OutOfRangeLow.
     *          Defines lower bound of sensor operating envelope. Typical values:
     *          - Ultrasonic sensors: 0.2m
     *          - Lidar sensors: 0.05m to 0.1m
     *          - Analog sensors: varies by model
     * 
     * @warning Critical for obstacle avoidance - readings below min_distance are
     *          considered invalid and not used for terrain following or collision prevention
     */
    AP_Float min_distance;
    
    /**
     * @brief Maximum valid measurement range (meters)
     * 
     * @details Distance readings above this threshold are rejected as OutOfRangeHigh.
     *          Defines upper bound of sensor operating envelope. Typical values:
     *          - Short-range ultrasonic: 5m to 8m
     *          - Medium-range lidar: 40m
     *          - Long-range lidar: 100m+
     * 
     * @warning Critical for obstacle avoidance - defines sensor operating envelope.
     *          Incorrect max_distance can cause navigation failures when terrain
     *          distance exceeds configured maximum
     */
    AP_Float max_distance;
    
    /**
     * @brief Rangefinder sensor type (RangeFinder::Type enum)
     * 
     * @details Selects sensor driver backend:
     *          - 0: None (disabled)
     *          - 1: Analog
     *          - 2: MaxBotix I2C
     *          - 3: LightWare I2C
     *          - 4: LightWare Serial
     *          - 5: PulsedLight I2C (LIDAR-Lite)
     *          - 6: PWM (sonar with PWM output)
     *          - 8: uBlox I2C
     *          - 10: MAVLink
     *          - 11: uLanding
     *          - 12: LeddarOne
     *          - 13: MaxBotix Serial
     *          - 14: TeraRanger I2C
     *          - 15: VL53L0X I2C
     *          - 16: NMEA (serial GPS with sonar)
     *          - 17: WASP (AirSpeed sensor)
     *          - 18: BenewakeTF02
     *          - 19: Benewake TFmini
     *          - 20: LightWareSerial (LW20)
     *          - 21: Leddar VU8 (serial)
     *          - 22: HC-SR04 (ultrasonic)
     *          - 23: NMEA (depth sounder)
     *          - 24: BenewakeTF03
     *          - 25: VL53L1X I2C
     *          - 26: BenewakeTFminiPlus I2C
     *          - 27: LightWare Serial (LW20c)
     *          - 28: TrueRange
     *          - 29: Nooploop TOFSense
     *          - 100: SITL (simulation)
     *          
     *          Changing type requires reboot to probe and initialize new sensor driver.
     * 
     * @note Full enum defined in libraries/AP_RangeFinder/AP_RangeFinder.h
     * 
     * @warning Parameter changes require reboot to detect and initialize sensor with
     *          new driver backend
     */
    AP_Int8  type;
    
    /**
     * @brief Analog pin number or GPIO pin for sensor interface
     * 
     * @details Usage depends on sensor type:
     *          - Analog rangefinders: ADC pin number (0-15 depending on board)
     *          - PWM rangefinders: GPIO pin receiving PWM signal
     *          - Digital sensors (I2C, Serial): Unused/ignored
     *          
     *          Pin assignments are board-specific and defined in hwdef files.
     */
    AP_Int8  pin;
    
    /**
     * @brief Ratiometric mode for analog rangefinders (boolean: 0=absolute, 1=ratiometric)
     * 
     * @details Selects analog voltage measurement mode:
     *          - 0: Absolute voltage (sensor referenced to board ground)
     *          - 1: Ratiometric (sensor referenced to ADC supply voltage)
     *          
     *          Ratiometric mode compensates for ADC reference voltage variations,
     *          improving accuracy when sensor and ADC share same power supply.
     * 
     * @note Only applicable to analog rangefinder types, ignored for digital sensors
     */
    AP_Int8  ratiometric;
    
    /**
     * @brief GPIO pin for sensor power control or enable signal
     * 
     * @details Output pin used to:
     *          - Power sensor on/off for power saving (stop_pin LOW = off, HIGH = on)
     *          - Enable/disable sensor operation
     *          - Control sensor reset line
     *          
     *          Set to -1 to disable (sensor always powered). Used in conjunction with
     *          powersave_range for automatic power management.
     * 
     * @note Enables power saving by shutting down sensor when not needed at high altitudes
     */
    AP_Int8  stop_pin;
    
    /**
     * @brief Analog transfer function type (RangeFinder::Function enum)
     * 
     * @details Defines voltage-to-distance conversion function for analog sensors:
     *          - 0: Linear (distance = voltage * scaling + offset)
     *          - 1: Inverted (distance inversely proportional to voltage)
     *          - 2: Hyperbola (1/distance proportional to voltage)
     *          
     *          Matches sensor datasheet transfer function for accurate distance calculation.
     * 
     * @note Only applicable to analog rangefinder types. Digital sensors use their
     *       native protocol for distance reporting
     */
    AP_Int8  function;
    
    /**
     * @brief Minimum altitude vehicle maintains above ground (meters)
     * 
     * @details Used for terrain following and surface tracking modes:
     *          - AUTO missions with terrain following enabled
     *          - Surface tracking in boats/subs
     *          - Precision landing altitude control
     *          
     *          Vehicle attempts to maintain this clearance when following terrain.
     *          Does not affect manual flight modes.
     * 
     * @note Critical parameter for terrain following safety - ensures adequate clearance
     *       over obstacles and rough terrain
     */
    AP_Float ground_clearance;
    
    /**
     * @brief I2C address, CAN node ID, or AP_Periph system ID
     * 
     * @details Usage depends on sensor communication protocol:
     *          - I2C sensors: 7-bit I2C address (typically 0x62, 0x29, etc.)
     *          - CAN/DroneCAN: CAN node ID
     *          - MAVLink/AP_Periph: System ID for remote rangefinder
     *          - Serial sensors: Unused/ignored
     *          
     *          Enables multiple sensors of same type on single bus by using different addresses.
     * 
     * @note Essential for multi-sensor configurations - allows addressing specific sensor
     *       when multiple identical sensors share same I2C bus or CAN network
     * 
     * @warning Incorrect address prevents sensor detection during initialization. Check
     *          sensor datasheet for default address and any address configuration options
     */
    AP_Int8  address;
    
    /**
     * @brief Sensor mounting orientation (enum Rotation)
     * 
     * @details Defines sensor pointing direction using standard rotation enum:
     *          - ROTATION_NONE (0): No rotation
     *          - ROTATION_YAW_45 (1): Rotated 45° right
     *          - ROTATION_YAW_90 (2): Rotated 90° right
     *          - ROTATION_YAW_135 (3): Rotated 135° right
     *          - ROTATION_YAW_180 (4): Rotated 180° (backward)
     *          - ROTATION_YAW_225 (5): Rotated 225° right
     *          - ROTATION_YAW_270 (6): Rotated 270° right (left)
     *          - ROTATION_YAW_315 (7): Rotated 315° right
     *          - ROTATION_ROLL_180 (8): Upside down
     *          - ROTATION_PITCH_180 (9): Pointing backward
     *          - ROTATION_ROLL_180_YAW_90 (10): Upside down and rotated 90° right
     *          - ROTATION_PITCH_270 (24): Pointing downward (most common for altitude)
     *          - Plus additional 3D rotation combinations
     *          
     *          Orientation critical for:
     *          - Object avoidance (forward/backward/side sensors)
     *          - Altitude hold (downward-facing sensor)
     *          - 360° obstacle detection arrays
     * 
     * @note Full Rotation enum defined in libraries/AP_Math/rotations.h
     * 
     * @warning Incorrect orientation causes distance measurements to be applied in wrong
     *          direction, potentially causing navigation errors or obstacle avoidance failures
     */
    AP_Int8  orientation;
};

#endif  // AP_RANGEFINDER_ENABLED
