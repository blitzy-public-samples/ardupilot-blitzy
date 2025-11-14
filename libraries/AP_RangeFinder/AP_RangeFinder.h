/**
 * @file AP_RangeFinder.h
 * @brief RangeFinder singleton manager coordinating multiple rangefinder backends
 *
 * @details This file implements the frontend manager for the ArduPilot rangefinder
 *          (distance sensor) subsystem. The RangeFinder class acts as a singleton
 *          that coordinates up to RANGEFINDER_MAX_INSTANCES (10) independent
 *          rangefinder sensors, each potentially of different types and orientations.
 *
 *          Architecture: Frontend/Backend pattern
 *          - Frontend (this class): Manages lifecycle, configuration, and provides
 *            unified interface for accessing distance measurements
 *          - Backends (AP_RangeFinder_Backend subclasses): Implement sensor-specific
 *            detection, initialization, and measurement logic
 *
 *          The manager handles:
 *          - Automatic sensor detection and backend instantiation during init()
 *          - Periodic sensor updates at ~10Hz via update() method
 *          - MAVLink and MSP protocol integration for remote rangefinders
 *          - Orientation-based sensor queries (e.g., downward-facing vs forward-facing)
 *          - Pre-arm health checks ensuring sensors provide valid data
 *          - Binary logging of rangefinder measurements
 *          - Parameter management for per-instance configuration
 *
 *          Typical usage: Vehicles query distance by orientation using
 *          distance_orient(), status_orient(), and has_data_orient() methods.
 *
 * @note Access via singleton: AP::rangefinder() or RangeFinder::get_singleton()
 * @note Called from main vehicle loop at ~10Hz for update()
 * @warning Thread safety: detect_sem semaphore protects backend registration during init()
 *
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include "AP_RangeFinder_Params.h"

// Maximum number of range finder instances available on this platform
#ifndef RANGEFINDER_MAX_INSTANCES 
  #if AP_RANGEFINDER_ENABLED
  #define RANGEFINDER_MAX_INSTANCES 10
  #else
  #define RANGEFINDER_MAX_INSTANCES 1
  #endif
#endif

#define RANGEFINDER_GROUND_CLEARANCE_DEFAULT 0.10
#define RANGEFINDER_PREARM_ALT_MAX_CM           200
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   0
#else
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   50
#endif

class AP_RangeFinder_Backend;

/**
 * @class RangeFinder
 * @brief Frontend manager for multiple rangefinder (distance sensor) instances
 *
 * @details RangeFinder is a singleton class that manages up to RANGEFINDER_MAX_INSTANCES
 *          (default 10) rangefinder sensors of potentially different types and orientations.
 *          This frontend provides a unified interface for sensor configuration, health
 *          monitoring, and distance queries, while delegating sensor-specific operations
 *          to backend driver classes.
 *
 *          Lifecycle:
 *          1. Construction: Singleton instantiated early in vehicle initialization
 *          2. init(): Detects configured sensors and instantiates appropriate backends
 *          3. update(): Called at ~10Hz from main loop, polls all backends for new data
 *          4. Query methods: Vehicles access distance/status by sensor orientation
 *
 *          Key Features:
 *          - Multi-sensor support: Up to 10 independent rangefinders simultaneously
 *          - Orientation-based queries: Find sensor by mounting direction (down, forward, etc.)
 *          - Protocol integration: Handles MAVLink DISTANCE_SENSOR and MSP rangefinder messages
 *          - Health monitoring: Pre-arm checks verify sensors provide changing valid data
 *          - Automatic detection: Probes configured sensor types during initialization
 *          - Binary logging: Records distance measurements for analysis
 *
 *          Frontend/Backend Architecture:
 *          - Frontend responsibilities: Parameter management, sensor registration,
 *            health checks, orientation-based lookup, logging
 *          - Backend responsibilities: Hardware-specific detection, communication
 *            protocol implementation, measurement acquisition, data validation
 *
 *          Configuration:
 *          - Parameters: RNGFNDx_TYPE (sensor type), RNGFNDx_ORIENT (mounting direction),
 *            RNGFNDx_MIN_CM/MAX_CM (valid range), RNGFNDx_ADDR (I2C address), etc.
 *          - See AP_RangeFinder_Params for complete parameter set
 *
 * @note Singleton access: Use AP::rangefinder() or RangeFinder::get_singleton()
 * @note Thread safety: detect_sem protects drivers[] array during backend registration
 * @warning Do not instantiate directly - use singleton accessor
 * @warning update() must be called at regular intervals (~10Hz) for sensors to function
 *
 * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:48-339
 */
class RangeFinder
{
    friend class AP_RangeFinder_Backend;
    //UAVCAN drivers are initialised in the Backend, hence list of drivers is needed there.
    friend class AP_RangeFinder_DroneCAN;
public:
    /**
     * @brief Construct RangeFinder singleton manager
     *
     * @details Initializes the rangefinder frontend with default state. Sets up
     *          parameter storage for all potential instances and prepares for
     *          sensor detection during init() call.
     *
     * @note Called automatically during vehicle initialization - do not call directly
     * @note Actual sensor detection and backend creation occurs in init()
     */
    RangeFinder();

    /* Do not allow copies */
    CLASS_NO_COPY(RangeFinder);

    /**
     * @enum Type
     * @brief Rangefinder sensor driver types supported by ArduPilot
     *
     * @details Enumerates all rangefinder (distance sensor) types that can be configured
     *          via RNGFNDx_TYPE parameter. Each type corresponds to a specific backend
     *          driver implementation handling sensor-specific communication protocols
     *          and measurement acquisition.
     *
     *          Sensor Communication Interfaces:
     *          - I2C: MBI2C, PLI2C, LWI2C, TRI2C, VL53L0X, VL53L1X_Short, TOFSenseF_I2C
     *          - Serial: LWSER, USD1_Serial, LEDDARONE, MBSER, LeddarVu8_Serial,
     *                    TeraRanger_Serial, JRE_Serial, RDS02UF
     *          - PWM: PX4_PWM, PWM, HC_SR04
     *          - CAN/DroneCAN: USD1_CAN, Benewake_CAN, TOFSenseP_CAN, NRA24_CAN, UAVCAN
     *          - Protocol: MAVLink (remote sensors), MSP, NMEA, Lua_Scripting
     *          - Platform-specific: ANALOG, BBB_PRU, BEBOP
     *
     *          Conditional Compilation:
     *          Most enum values are conditionally compiled based on AP_RANGEFINDER_*_ENABLED
     *          feature flags defined in AP_RangeFinder_config.h. This allows excluding
     *          unused drivers to save flash memory on resource-constrained boards.
     *
     *          Common Types:
     *          - VL53L0X/VL53L1X_Short: STMicroelectronics time-of-flight I2C sensors
     *          - LightWare (LWI2C, LWSER): Laser rangefinders (I2C and serial variants)
     *          - Benewake TF series: TF02, TFmini, TFminiPlus, TF03 lidar sensors
     *          - MaxBotix (MBI2C, MBSER): Ultrasonic sensors (I2C and serial variants)
     *          - MAVLink: Remote rangefinders via MAVLink DISTANCE_SENSOR messages
     *          - Lua_Scripting: Custom rangefinder implemented in Lua scripts
     *
     * @note Type::NONE (0) indicates no sensor configured for that instance
     * @note SIM (100) used only in SITL (Software In The Loop) simulation
     * @warning Enum values must match RNGFNDx_TYPE parameter definitions
     * @warning Changing enum values breaks parameter compatibility with ground stations
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:60-191
     */
    enum class Type {
        NONE   = 0,  ///< No rangefinder configured
#if AP_RANGEFINDER_ANALOG_ENABLED
        ANALOG = 1,  ///< Analog voltage input rangefinder
#endif
#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
        MBI2C  = 2,  ///< MaxBotix I2C ultrasonic sensor (MaxSonar I2CXL series)
#endif
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        PLI2C  = 3,  ///< PulsedLight I2C laser rangefinder (LIDAR-Lite v2)
#endif
//        PX4    = 4, // no longer used, but may be in some user's parameters
#if AP_RANGEFINDER_PWM_ENABLED
        PX4_PWM= 5,  ///< PWM-based rangefinder (legacy PX4 sonar)
#endif
#if AP_RANGEFINDER_BBB_PRU_ENABLED
        BBB_PRU= 6,  ///< BeagleBone Black PRU ultrasonic sensor
#endif
#if AP_RANGEFINDER_LWI2C_ENABLED
        LWI2C  = 7,  ///< LightWare I2C laser rangefinder
#endif
#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
        LWSER  = 8,  ///< LightWare Serial laser rangefinder
#endif
#if AP_RANGEFINDER_BEBOP_ENABLED
        BEBOP  = 9,  ///< Parrot Bebop built-in downward ultrasonic sensor
#endif
#if AP_RANGEFINDER_MAVLINK_ENABLED
        MAVLink = 10, ///< Remote rangefinder via MAVLink DISTANCE_SENSOR messages
#endif
#if AP_RANGEFINDER_USD1_SERIAL_ENABLED
        USD1_Serial = 11, ///< USD1 Serial ultrasonic sensor
#endif
#if AP_RANGEFINDER_LEDDARONE_ENABLED
        LEDDARONE = 12, ///< LeddarOne laser rangefinder
#endif
#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
        MBSER  = 13, ///< MaxBotix Serial ultrasonic sensor
#endif
#if AP_RANGEFINDER_TRI2C_ENABLED
        TRI2C  = 14, ///< TeraRanger I2C laser rangefinder
#endif
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        PLI2CV3= 15, ///< PulsedLight I2C v3 laser rangefinder (LIDAR-Lite v3)
#endif
        VL53L0X = 16, ///< STMicroelectronics VL53L0X time-of-flight I2C sensor
#if AP_RANGEFINDER_NMEA_ENABLED
        NMEA = 17, ///< NMEA protocol depth sounder (underwater/marine)
#endif
#if AP_RANGEFINDER_WASP_ENABLED
        WASP = 18, ///< Wasp laser rangefinder
#endif
#if AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
        BenewakeTF02 = 19, ///< Benewake TF02 lidar sensor
#endif
#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
        BenewakeTFmini = 20, ///< Benewake TFmini lidar sensor
#endif
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        PLI2CV3HP = 21, ///< PulsedLight I2C v3HP laser rangefinder (high performance)
#endif
#if AP_RANGEFINDER_PWM_ENABLED
        PWM = 22, ///< Generic PWM rangefinder
#endif
#if AP_RANGEFINDER_BLPING_ENABLED
        BLPing = 23, ///< BlueRobotics Ping underwater sonar
#endif
#if AP_RANGEFINDER_DRONECAN_ENABLED
        UAVCAN = 24, ///< DroneCAN/UAVCAN distance sensor
#endif
#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
        BenewakeTFminiPlus = 25, ///< Benewake TFmini Plus lidar sensor
#endif
#if AP_RANGEFINDER_LANBAO_ENABLED
        Lanbao = 26, ///< Lanbao laser rangefinder
#endif
#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
        BenewakeTF03 = 27, ///< Benewake TF03 long-range lidar sensor
#endif
        VL53L1X_Short = 28, ///< STMicroelectronics VL53L1X time-of-flight I2C sensor (short range mode)
#if AP_RANGEFINDER_LEDDARVU8_ENABLED
        LeddarVu8_Serial = 29, ///< LeddarVu8 8-segment laser rangefinder (serial)
#endif
#if AP_RANGEFINDER_HC_SR04_ENABLED
        HC_SR04 = 30, ///< HC-SR04 ultrasonic sensor (GPIO trigger/echo)
#endif
#if AP_RANGEFINDER_GYUS42V2_ENABLED
        GYUS42v2 = 31, ///< DFRobot URM series ultrasonic sensor (I2C)
#endif
#if HAL_MSP_RANGEFINDER_ENABLED
        MSP = 32, ///< MSP (MultiWii Serial Protocol) rangefinder
#endif
#if AP_RANGEFINDER_USD1_CAN_ENABLED
        USD1_CAN = 33, ///< USD1 ultrasonic sensor via CAN bus
#endif
#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
        Benewake_CAN = 34, ///< Benewake lidar sensor via CAN bus
#endif
#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
        TeraRanger_Serial = 35, ///< TeraRanger laser rangefinder (serial)
#endif
#if AP_RANGEFINDER_LUA_ENABLED
        Lua_Scripting = 36, ///< Custom rangefinder implemented via Lua scripting
#endif
#if AP_RANGEFINDER_NOOPLOOP_ENABLED
        NoopLoop_P = 37, ///< Nooploop TOF_P rangefinder
#endif
#if AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED
        TOFSenseP_CAN = 38, ///< TOFSense-P CAN bus rangefinder
#endif
#if AP_RANGEFINDER_NRA24_CAN_ENABLED
        NRA24_CAN = 39, ///< NRA24 24GHz radar via CAN bus
#endif
#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
        TOFSenseF_I2C = 40, ///< TOFSense-F I2C rangefinder
#endif
#if AP_RANGEFINDER_JRE_SERIAL_ENABLED
        JRE_Serial = 41, ///< JRE Serial rangefinder
#endif
#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
        Ainstein_LR_D1 = 42, ///< Ainstein LR-D1 long-range radar
#endif
#if AP_RANGEFINDER_RDS02UF_ENABLED
        RDS02UF = 43, ///< RDS02UF ultrasonic sensor (serial)
#endif
#if AP_RANGEFINDER_HEXSOONRADAR_ENABLED
        HEXSOON_RADAR = 44, ///< Hexsoon radar rangefinder
#endif
#if AP_RANGEFINDER_SIM_ENABLED
        SIM = 100, ///< Simulated rangefinder for SITL (Software In The Loop)
#endif
    };

    /**
     * @enum Function
     * @brief Transfer function for analog voltage rangefinder conversion
     *
     * @details Defines the mathematical relationship between analog voltage input
     *          and distance measurement for ANALOG type rangefinders. Used to
     *          convert raw ADC voltage readings to distance values.
     *
     *          Transfer Functions:
     *          - LINEAR: distance = (voltage - offset) * scaling
     *          - INVERTED: distance = scaling / (voltage - offset)
     *          - HYPERBOLA: distance = scaling / (voltage - offset)^2
     *
     * @note Only applicable to Type::ANALOG rangefinders
     * @note Configured via RNGFNDx_FUNCTION parameter
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:193-197
     */
    enum class Function {
        LINEAR    = 0, ///< Linear voltage-to-distance relationship
        INVERTED  = 1, ///< Inverse voltage-to-distance relationship
        HYPERBOLA = 2  ///< Hyperbolic voltage-to-distance relationship
    };

    /**
     * @enum Status
     * @brief Rangefinder sensor health and data validity status
     *
     * @details Indicates the current state of a rangefinder sensor, progressing
     *          from NotConnected (sensor not detected) through NoData (detected
     *          but no measurements) to Good (valid measurements within range).
     *          Out-of-range conditions are distinguished as too-close vs too-far.
     *
     *          Status Progression (typical sensor lifecycle):
     *          NotConnected → NoData → OutOfRangeLow/OutOfRangeHigh/Good
     *
     *          Usage in Health Monitoring:
     *          - Pre-arm checks require Status::Good with changing distance values
     *          - Flight code should check status_orient() before using distance
     *          - OutOfRange conditions may trigger failsafes depending on application
     *
     *          Status Meanings:
     *          - NotConnected: Sensor not detected during init or communication lost
     *          - NoData: Sensor detected but no valid measurement received yet
     *          - OutOfRangeLow: Object too close (below RNGFNDx_MIN_CM)
     *          - OutOfRangeHigh: No object detected (beyond RNGFNDx_MAX_CM)
     *          - Good: Valid distance measurement within configured range
     *
     * @note Status stored in RangeFinder_State::status field for each instance
     * @note Query via status_orient() for orientation-based sensor lookup
     * @warning OutOfRangeLow vs OutOfRangeHigh distinction critical for terrain following
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:199-205
     */
    enum class Status {
        NotConnected   = 0, ///< Sensor not detected or communication failure
        NoData         = 1, ///< Sensor detected but no measurement received
        OutOfRangeLow  = 2, ///< Object detected too close (below minimum range)
        OutOfRangeHigh = 3, ///< No object detected (beyond maximum range)
        Good           = 4, ///< Valid distance measurement within configured range
    };

    /// Minimum valid signal quality percentage (0%)
    static constexpr int8_t SIGNAL_QUALITY_MIN = 0;
    /// Maximum valid signal quality percentage (100%)
    static constexpr int8_t SIGNAL_QUALITY_MAX = 100;
    /// Signal quality unknown or not supported by sensor
    static constexpr int8_t SIGNAL_QUALITY_UNKNOWN = -1;

    /**
     * @struct RangeFinder_State
     * @brief Per-instance sensor state filled by backend drivers
     *
     * @details Contains the current measurement data and health status for a single
     *          rangefinder instance. Backend drivers populate this structure during
     *          their update() method, and the frontend provides access via
     *          orientation-based query methods.
     *
     *          Update Frequency:
     *          State is updated at sensor-specific rates (typically 10-100Hz) by
     *          backends, with frontend update() method called at ~10Hz from main loop.
     *
     *          Distance Representation:
     *          - Primary: distance_m in meters (use this for all new code)
     *          - Legacy: Centimeter accessors exist for scripting compatibility
     *
     *          Signal Quality:
     *          Not all sensors provide quality metric. Check signal_quality_pct:
     *          - SIGNAL_QUALITY_UNKNOWN (-1): Sensor doesn't report quality
     *          - 0-100: Quality percentage (100 = excellent, 0 = poor but valid)
     *
     *          Health Monitoring:
     *          - range_valid_count tracks consecutive valid readings (max 10)
     *          - last_reading_ms used to detect sensor timeouts
     *          - status indicates current sensor health (see Status enum)
     *
     * @note Backends must update last_reading_ms on every successful sensor read
     * @note Frontend zeros distance_m if status != Good to prevent stale data usage
     * @warning Always check status field before using distance_m in flight code
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:212-221
     */
    struct RangeFinder_State {
        float distance_m;               ///< Distance measurement in meters
        int8_t signal_quality_pct;      ///< Measurement quality 0-100%, or -1 if unknown
        uint16_t voltage_mv;            ///< Sensor voltage in millivolts (analog sensors), 0 if N/A
        enum RangeFinder::Status status; ///< Sensor health and data validity status
        uint8_t  range_valid_count;     ///< Consecutive valid readings count (maxes at 10)
        uint32_t last_reading_ms;       ///< System time (AP_HAL::millis()) of last successful update

        const struct AP_Param::GroupInfo *var_info; ///< Parameter metadata for this instance
    };

    /**
     * @brief Backend-specific parameter group information pointers
     *
     * @details Array of parameter metadata pointers for backend-specific parameters.
     *          Each backend driver type can register additional parameters beyond
     *          the common frontend parameters (e.g., I2C address, baud rate, etc.).
     *
     * @note Populated by backends during their constructor
     * @note Used by AP_Param system for parameter discovery and storage
     */
    static const struct AP_Param::GroupInfo *backend_var_info[RANGEFINDER_MAX_INSTANCES];

    /**
     * @brief Frontend parameter group information for AP_Param registration
     *
     * @details Defines the parameter table for RangeFinder frontend, including
     *          common parameters like RNGFNDx_TYPE, RNGFNDx_ORIENT, RNGFNDx_MIN_CM,
     *          RNGFNDx_MAX_CM, etc. See AP_RangeFinder_Params for complete list.
     *
     * @note Used during AP_Param::setup() for parameter system initialization
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Configure logging bitmask for RFND messages
     *
     * @details Sets the log bitmask bit that enables/disables rangefinder (RFND)
     *          message logging. Called during vehicle initialization to configure
     *          which log messages are recorded to dataflash/SD card.
     *
     * @param[in] log_rfnd_bit Bitmask value for RFND logging enable/disable
     *
     * @note Typically called once during vehicle setup
     * @note RFND log messages contain distance, status, and signal quality
     */
    void set_log_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    /**
     * @brief Get count of configured rangefinder instances
     *
     * @details Returns the number of rangefinder instances that should be
     *          considered when iterating over sensors. This is the maximum
     *          sensor ID plus one, not necessarily the count of actually
     *          configured sensors.
     *
     *          Example: If RNGFNDx_TYPE is set for instances 0, 1, and 4,
     *          this returns 5 (not 3), allowing iteration over all potential
     *          slots including gaps.
     *
     *          Typical usage:
     *          @code
     *          for (uint8_t i = 0; i < rangefinder->num_sensors(); i++) {
     *              AP_RangeFinder_Backend *backend = rangefinder->get_backend(i);
     *              if (backend != nullptr) {
     *                  // Process sensor i
     *              }
     *          }
     *          @endcode
     *
     * @return Maximum sensor index + 1, suitable for iteration bounds
     *
     * @note Not the count of active sensors - gaps may exist in configuration
     * @note Updated during init() based on configured RNGFNDx_TYPE parameters
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:236-238
     */
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    /**
     * @brief Pre-arm health check for all configured rangefinders
     *
     * @details Verifies that all configured rangefinders are healthy and providing
     *          valid, changing distance measurements. Required checks:
     *          - Sensor status must be Good
     *          - Distance must change by at least RANGEFINDER_PREARM_REQUIRED_CHANGE_CM
     *            (50cm default, 0cm in SITL) when sensor is moved
     *          - All readings must be within configured min/max range
     *
     *          Purpose: Ensures sensors are working before arming to prevent
     *          in-flight failures during terrain following, landing, or obstacle
     *          avoidance operations.
     *
     * @param[out] failure_msg Buffer to receive failure description if check fails
     * @param[in]  failure_msg_len Size of failure_msg buffer in bytes
     *
     * @return true if all configured rangefinders pass health checks
     * @return false if any sensor fails, with reason in failure_msg
     *
     * @note Called by vehicle arming checks before allowing flight
     * @note Requires distance variation to pass - static readings fail the check
     * @warning Sensor must show movement during pre-arm to pass (anti-stuck check)
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:241
     */
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    /**
     * @brief Detect and initialize all configured rangefinder backends
     *
     * @details Scans RNGFNDx_TYPE parameters for configured sensors, attempts to
     *          detect each sensor type, and instantiates appropriate backend drivers.
     *          Called once during vehicle initialization.
     *
     *          Detection Process:
     *          1. Iterate through RANGEFINDER_MAX_INSTANCES parameter slots
     *          2. For each configured Type, call detect_instance()
     *          3. Backend probe() methods attempt hardware communication
     *          4. Successfully detected backends registered via _add_backend()
     *          5. Backends initialize and start periodic measurement updates
     *
     *          Backend Instantiation:
     *          - I2C sensors: Probe I2C bus at configured address
     *          - Serial sensors: Open configured serial port
     *          - Protocol sensors: Register message handlers (MAVLink, MSP)
     *          - Platform sensors: Initialize hardware-specific interfaces
     *
     * @param[in] orientation_default Default mounting orientation for sensors
     *                                 without explicit RNGFNDx_ORIENT parameter
     *
     * @note Called once from vehicle setup() - do not call repeatedly
     * @note Thread safety: detect_sem semaphore protects backend registration
     * @warning Backends not successfully detected will be nullptr in drivers[] array
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:244
     */
    void init(enum Rotation orientation_default);

    /**
     * @brief Update all rangefinder backend measurements
     *
     * @details Polls all instantiated backend drivers for new distance measurements.
     *          Each backend's update() method acquires sensor data, validates it,
     *          and updates its RangeFinder_State. Frontend then logs measurements
     *          if configured.
     *
     *          Call Frequency: ~10Hz from vehicle main loop
     *
     *          Update Process:
     *          1. Iterate through all backends in drivers[] array
     *          2. Call each backend's update() method
     *          3. Backend reads sensor, updates state (distance_m, status, etc.)
     *          4. Frontend logs RFND messages if logging enabled
     *
     *          Backend Update Behavior:
     *          - Serial/I2C: Poll for new data, parse protocol, validate range
     *          - Protocol (MAVLink/MSP): Process queued messages from handle_msg()
     *          - Simulation: Update from simulated sensor model
     *
     * @note Must be called at regular intervals (~10Hz) for sensors to function
     * @note Backends may internally rate-limit sensor queries to match hardware
     * @note Frontend zeros distance if backend reports status != Good
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:247-248
     */
    void update(void);

    /**
     * @brief Process incoming MAVLink DISTANCE_SENSOR message
     *
     * @details Handles DISTANCE_SENSOR messages from MAVLink-enabled rangefinders,
     *          typically remote sensors on companion computers or external devices.
     *          Extracts distance, signal quality, and status, then updates the
     *          appropriate backend's state.
     *
     *          Message Flow:
     *          1. GCS_MAVLink receives DISTANCE_SENSOR message
     *          2. Calls this handler with message
     *          3. Finds MAVLink backend by sensor ID
     *          4. Backend processes and validates measurement
     *          5. State updated for query by vehicle code
     *
     * @param[in] msg MAVLink message containing DISTANCE_SENSOR payload
     *
     * @note Requires Type::MAVLink backend configured for this sensor ID
     * @note Multiple MAVLink rangefinders supported with different sensor IDs
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:251
     */
    void handle_msg(const mavlink_message_t &msg);

#if HAL_MSP_RANGEFINDER_ENABLED
    /**
     * @brief Process incoming MSP rangefinder data packet
     *
     * @details Handles MSP (MultiWii Serial Protocol) rangefinder messages,
     *          typically from flight controllers running MSP-compatible firmware
     *          that provide rangefinder data over serial link.
     *
     * @param[in] pkt MSP rangefinder data message packet
     *
     * @note Requires Type::MSP backend configured
     * @note Only compiled if HAL_MSP_RANGEFINDER_ENABLED is set
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:254-255
     */
    void handle_msp(const MSP::msp_rangefinder_data_message_t &pkt);
#endif

    /**
     * @brief Check if rangefinder with specified orientation exists and has data
     *
     * @details Searches configured rangefinders for one matching the specified
     *          mounting orientation (e.g., ROTATION_PITCH_270 for downward-facing).
     *          Verifies sensor is both configured and currently providing valid data.
     *
     * @param[in] orientation Desired sensor mounting direction (Rotation enum)
     *
     * @return true if sensor with this orientation exists and status is Good
     * @return false if no matching sensor or sensor unhealthy
     *
     * @note Use before calling distance_orient() to verify sensor availability
     * @note Faster than distance_orient() for existence check only
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:258
     */
    bool has_orientation(enum Rotation orientation) const;

    /**
     * @brief Find first rangefinder backend with specified orientation
     *
     * @details Searches drivers[] array for first backend matching the requested
     *          mounting orientation. Useful when direct backend access is needed
     *          rather than just distance query.
     *
     * @param[in] orientation Desired sensor mounting direction (Rotation enum)
     *
     * @return Pointer to backend driver if found, nullptr if not found
     *
     * @note Returns first match - multiple sensors with same orientation possible
     * @note Prefer orientation-based query methods (distance_orient, etc.) for data access
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:261
     */
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    /**
     * @brief Get rangefinder backend driver by instance ID
     *
     * @details Direct accessor to backend drivers array by zero-based instance ID.
     *          Instance IDs correspond to RNGFNDx parameter indices (0-9).
     *
     * @param[in] id Instance ID (0 to RANGEFINDER_MAX_INSTANCES-1)
     *
     * @return Pointer to backend driver if configured, nullptr if unconfigured or invalid ID
     *
     * @note Prefer orientation-based access unless you need specific instance
     * @note Returns nullptr for gaps in configuration (e.g., if only RNGFND0 and RNGFND3 configured)
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:263
     */
    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    /**
     * @brief Get configured sensor type for instance ID
     *
     * @details Returns the Type enum value for the specified rangefinder instance,
     *          read from RNGFNDx_TYPE parameter. Returns Type::NONE if instance
     *          unconfigured or ID invalid.
     *
     * @param[in] id Instance ID (0 to RANGEFINDER_MAX_INSTANCES-1)
     *
     * @return Type enum value for this instance, Type::NONE if unconfigured
     *
     * @note Returns parameter value even if backend failed to initialize
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:266-268
     */
    Type get_type(uint8_t id) const {
        return id >= RANGEFINDER_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    /**
     * @brief Get I2C/CAN address for instance (AP_Periph usage)
     *
     * @details Returns RNGFNDx_ADDR parameter value, used for I2C address
     *          selection or AP_Periph CAN node addressing. Returns 0 if
     *          instance invalid or address not applicable.
     *
     * @param[in] id Instance ID (0 to RANGEFINDER_MAX_INSTANCES-1)
     *
     * @return Configured address value, 0 if invalid/unconfigured
     *
     * @note Meaning of address depends on sensor type (I2C address, CAN node ID, etc.)
     * @note Used primarily by AP_Periph for CAN rangefinder node identification
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:271-273
     */
    uint8_t get_address(uint8_t id) const {
        return id >= RANGEFINDER_MAX_INSTANCES? 0 : uint8_t(params[id].address.get());
    }

    /**
     * @brief Get signal quality percentage for sensor with specified orientation
     *
     * @details Returns measurement quality metric for the first sensor matching
     *          the specified orientation. Quality interpretation:
     *          - SIGNAL_QUALITY_UNKNOWN (-1): Sensor doesn't report quality
     *          - 0-100: Quality percentage (100 = excellent, 0 = poor but valid)
     *
     * @param[in] orientation Desired sensor mounting direction (Rotation enum)
     *
     * @return Signal quality 0-100%, or SIGNAL_QUALITY_UNKNOWN (-1) if not supported
     * @return SIGNAL_QUALITY_UNKNOWN if no sensor with this orientation
     *
     * @note Not all sensor types provide quality metric
     * @note Quality affects decision-making in sensor fusion and failsafe logic
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:277
     */
    int8_t signal_quality_pct_orient(enum Rotation orientation) const;

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Get distance in centimeters (DEPRECATED - use distance_orient)
     *
     * @deprecated Use distance_orient() for new code - meter accessors preferred
     *
     * @details Legacy centimeter accessor maintained for scripting compatibility.
     *          Returns distance measurement from first sensor matching orientation.
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Distance in centimeters, 0 if no valid measurement
     *
     * @note Only compiled when AP_SCRIPTING_ENABLED for backward compatibility
     * @warning Prefer meter-based distance_orient() for new vehicle code
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:280-282
     */
    uint16_t distance_cm_orient(enum Rotation orientation) const {
        return distance_orient(orientation) * 100.0;
    }

    /**
     * @brief Get maximum range in centimeters (DEPRECATED)
     * @deprecated Use max_distance_orient() instead
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:283-285
     */
    int32_t max_distance_cm_orient(enum Rotation orientation) const {
        return max_distance_orient(orientation) * 100;
    }

    /**
     * @brief Get minimum range in centimeters (DEPRECATED)
     * @deprecated Use min_distance_orient() instead
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:286-288
     */
    int32_t min_distance_cm_orient(enum Rotation orientation) const {
        return min_distance_orient(orientation) * 100;
    }

    /**
     * @brief Get ground clearance in centimeters (DEPRECATED)
     * @deprecated Use ground_clearance_orient() instead
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:289-291
     */
    int32_t ground_clearance_cm_orient(enum Rotation orientation) const {
        return ground_clearance_orient(orientation) * 100;
    }
#endif

    /**
     * @brief Get distance measurement in meters (PREFERRED)
     *
     * @details Returns current distance measurement from first sensor matching
     *          the specified orientation. This is the primary method for accessing
     *          rangefinder data in vehicle code.
     *
     *          Distance Validation:
     *          - Returns 0.0 if sensor status != Good
     *          - Returns 0.0 if no sensor with this orientation
     *          - Otherwise returns last valid measurement in meters
     *
     *          Typical usage:
     *          @code
     *          if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
     *              float distance_m = rangefinder->distance_orient(ROTATION_PITCH_270);
     *              // Use distance for terrain following, landing, etc.
     *          }
     *          @endcode
     *
     * @param[in] orientation Desired sensor mounting direction (Rotation enum)
     *
     * @return Distance in meters, 0.0 if no valid measurement or sensor not found
     *
     * @note ROTATION_PITCH_270 is downward-facing (most common for terrain following)
     * @note Always check has_data_orient() or status_orient() before using
     * @note Prefer this over deprecated distance_cm_orient() centimeter accessor
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:294
     */
    float distance_orient(enum Rotation orientation) const;

    /**
     * @brief Get maximum detection range in meters
     *
     * @details Returns RNGFNDx_MAX_CM parameter value (converted to meters) for
     *          first sensor matching orientation. Measurements beyond this range
     *          result in Status::OutOfRangeHigh.
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Maximum range in meters, 0.0 if sensor not found
     *
     * @note Used for range validation and flight planning
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:295
     */
    float max_distance_orient(enum Rotation orientation) const;

    /**
     * @brief Get minimum detection range in meters
     *
     * @details Returns RNGFNDx_MIN_CM parameter value (converted to meters) for
     *          first sensor matching orientation. Measurements below this range
     *          result in Status::OutOfRangeLow.
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Minimum range in meters, 0.0 if sensor not found
     *
     * @note Objects closer than minimum range may not be detected reliably
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:296
     */
    float min_distance_orient(enum Rotation orientation) const;

    /**
     * @brief Get configured ground clearance in meters
     *
     * @details Returns RNGFNDx_GNDCLEAR parameter value (default 0.10m) for
     *          first sensor matching orientation. Ground clearance represents
     *          the distance from rangefinder to ground when vehicle is landed.
     *
     *          Used to convert rangefinder distance to altitude above ground:
     *          altitude_agl = distance - ground_clearance
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Ground clearance in meters, RANGEFINDER_GROUND_CLEARANCE_DEFAULT if not found
     *
     * @note Critical for accurate landing altitude calculation
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:297
     */
    float ground_clearance_orient(enum Rotation orientation) const;

    /**
     * @brief Get MAVLink sensor type for specified orientation
     *
     * @details Returns MAV_DISTANCE_SENSOR enum value indicating sensor technology
     *          (laser, ultrasound, infrared, radar). Used for MAVLink telemetry
     *          reporting and ground station display.
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return MAVLink distance sensor type enum
     *
     * @note Each backend maps its sensor type to appropriate MAVLink enum
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:298
     */
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;

    /**
     * @brief Get sensor health status for specified orientation
     *
     * @details Returns Status enum indicating current sensor health. Check this
     *          before using distance measurement to ensure data validity.
     *
     *          Status Meanings:
     *          - NotConnected: Sensor not detected or communication lost
     *          - NoData: Sensor detected but no measurement received
     *          - OutOfRangeLow: Object too close (below minimum range)
     *          - OutOfRangeHigh: No object detected (beyond maximum range)
     *          - Good: Valid distance measurement available
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Status enum value, Status::NotConnected if sensor not found
     *
     * @note Only use distance when status == Status::Good
     * @warning OutOfRangeLow vs OutOfRangeHigh distinction matters for terrain following
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:299
     */
    RangeFinder::Status status_orient(enum Rotation orientation) const;

    /**
     * @brief Check if sensor has valid data for specified orientation
     *
     * @details Convenience method checking if sensor exists and status is Good.
     *          Equivalent to: status_orient(orientation) == Status::Good
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return true if sensor found and status is Good
     * @return false if sensor not found or unhealthy
     *
     * @note Faster check than status_orient() when only validity matters
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:300
     */
    bool has_data_orient(enum Rotation orientation) const;

    /**
     * @brief Get consecutive valid reading count for specified orientation
     *
     * @details Returns number of consecutive measurements with status Good,
     *          maxing out at 10. Used to detect sensor stability and filter
     *          transient errors. Reset to 0 when status != Good.
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Consecutive valid reading count (0-10), 0 if sensor not found
     *
     * @note Higher count indicates more stable/reliable measurements
     * @note Maxes at 10 to prevent overflow
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:301
     */
    uint8_t range_valid_count_orient(enum Rotation orientation) const;

    /**
     * @brief Get position offset vector for specified orientation
     *
     * @details Returns RNGFNDx_POS_X/Y/Z offset vector (meters) from vehicle
     *          center of gravity to rangefinder sensor. Used for accurate
     *          distance-to-terrain calculation accounting for sensor mounting location.
     *
     *          Coordinate System: Body frame (Forward-Right-Down)
     *          - X: Forward (positive = ahead of CG)
     *          - Y: Right (positive = right of CG)
     *          - Z: Down (positive = below CG)
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return Reference to position offset Vector3f in meters (body frame)
     * @return Reference to zero vector if sensor not found
     *
     * @note Used by EKF for accurate sensor fusion
     * @note Critical for precision landing and terrain following
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:302
     */
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;

    /**
     * @brief Get timestamp of last successful sensor reading
     *
     * @details Returns system time (AP_HAL::millis()) of last update where
     *          sensor provided new data. Used to detect sensor timeouts and
     *          data freshness.
     *
     *          Timeout Detection Example:
     *          @code
     *          uint32_t last_ms = rangefinder->last_reading_ms(ROTATION_PITCH_270);
     *          if (AP_HAL::millis() - last_ms > 500) {
     *              // Sensor timeout - no data for 500ms
     *          }
     *          @endcode
     *
     * @param[in] orientation Desired sensor mounting direction
     *
     * @return System time in milliseconds of last reading, 0 if sensor not found
     *
     * @note Updated even if measurement out of range (status != Good)
     * @note Use to detect communication loss vs out-of-range condition
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:303
     */
    uint32_t last_reading_ms(enum Rotation orientation) const;

    /**
     * @brief Get temperature reading from rangefinder sensor
     *
     * @details Some rangefinder sensors provide temperature measurements along
     *          with distance data. Temperature reading can be useful for sensor
     *          calibration or environmental monitoring. Not all sensor types
     *          support temperature reporting.
     *
     * @param[in]  orientation Desired sensor mounting direction
     * @param[out] temp        Temperature in degrees Celsius (populated on success)
     *
     * @return true if temperature reading available and temp populated
     * @return false if sensor doesn't support temperature or sensor not found
     *
     * @note Temperature reading independent of distance measurement status
     * @note Primarily supported by I2C time-of-flight sensors (VL53L0X, VL53L1X)
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:306
     */
    bool get_temp(enum Rotation orientation, float &temp) const;

    /**
     * @brief Set externally estimated terrain height for power optimization
     *
     * @details Provides rangefinder backends with estimated altitude above terrain
     *          from other sources (barometer, GPS, terrain database). Some sensors
     *          can use this to optimize power consumption or measurement range at
     *          high altitudes where terrain is far below.
     *
     *          Use Case: At cruise altitude (e.g., 100m AGL), downward rangefinder
     *          may be out of range. Backends can reduce measurement attempts or
     *          enter low-power mode until altitude decreases.
     *
     * @param[in] height Estimated height above terrain in meters
     *
     * @note Optional optimization - not required for basic operation
     * @note Not all backends utilize this information
     * @note Typically called from terrain following or landing code
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:312-314
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }

    /**
     * @brief Get singleton instance of RangeFinder
     *
     * @details Returns pointer to the global RangeFinder singleton instance.
     *          This is the standard method for accessing the rangefinder manager
     *          from vehicle code.
     *
     * @return Pointer to RangeFinder singleton, nullptr if not instantiated
     *
     * @note Prefer AP::rangefinder() accessor for cleaner syntax
     * @note Singleton created during vehicle initialization
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:316
     */
    static RangeFinder *get_singleton(void) { return _singleton; }

protected:
    /**
     * @brief Per-instance parameter storage for all rangefinder instances
     *
     * @details Array of AP_RangeFinder_Params structures holding configuration
     *          parameters for each of the potential RANGEFINDER_MAX_INSTANCES (10)
     *          rangefinder slots. Parameters include type, orientation, min/max
     *          range, I2C address, pin assignments, etc.
     *
     * @note Protected to allow backend access to parameter values
     * @note Indexed by instance ID (0-9) corresponding to RNGFNDx parameters
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:319
     */
    AP_RangeFinder_Params params[RANGEFINDER_MAX_INSTANCES];

private:
    /// Singleton instance pointer for global access
    static RangeFinder *_singleton;

    /// Per-instance sensor state updated by backends (distance, status, quality, etc.)
    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];

    /// Backend driver pointers, nullptr if instance not configured or detection failed
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];

    /// Count of configured instances (maximum instance index + 1)
    uint8_t num_instances;

    /// Semaphore protecting drivers[] array during backend registration in init()
    HAL_Semaphore detect_sem;

    /// Externally estimated terrain height for backend power optimization (meters)
    float estimated_terrain_height;

    /// Zero vector returned by get_pos_offset_orient() for invalid requests
    Vector3f pos_offset_zero;

    /**
     * @brief Convert legacy parameter formats to current schema
     *
     * @details Handles parameter migration from older ArduPilot versions.
     *          Called during init() to ensure parameter compatibility.
     *
     * @note Automatically updates stored parameters if conversion needed
     */
    void convert_params(void);

    /**
     * @brief Detect and instantiate backend for specific instance
     *
     * @details Attempts to detect rangefinder hardware for the specified instance
     *          based on configured Type parameter. Creates appropriate backend
     *          driver if detection successful.
     *
     * @param[in]     instance        Instance ID to detect (0-9)
     * @param[in,out] serial_instance Counter for serial port allocation across instances
     *
     * @note Called by init() for each configured instance
     * @note Thread-safe via detect_sem semaphore
     */
    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    /**
     * @brief Register successfully detected backend driver
     *
     * @details Adds backend pointer to drivers[] array and increments num_instances
     *          if this is highest index seen. Protected by detect_sem semaphore.
     *
     * @param[in] driver         Pointer to successfully instantiated backend
     * @param[in] instance       Instance ID for this backend (0-9)
     * @param[in] serial_instance Serial port index if serial-based sensor (default 0)
     *
     * @return true if backend successfully registered
     * @return false if registration failed (e.g., invalid instance ID)
     *
     * @note Called by detect_instance() after successful backend probe
     */
    bool _add_backend(AP_RangeFinder_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    /// Logging bitmask bit for enabling/disabling RFND message logging
    uint32_t _log_rfnd_bit = -1;

    /**
     * @brief Log rangefinder measurements to dataflash
     *
     * @details Writes RFND log messages containing distance, status, and signal
     *          quality for all active sensors. Called from update() if logging enabled.
     *
     * @note Logging controlled by _log_rfnd_bit bitmask
     * @note Multiple RFND messages logged (one per active instance)
     */
    void Log_RFND() const;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get RangeFinder singleton instance (preferred accessor)
     *
     * @details Preferred method for accessing RangeFinder singleton from vehicle code.
     *          Provides cleaner syntax than RangeFinder::get_singleton().
     *
     *          Usage example:
     *          @code
     *          RangeFinder *rfnd = AP::rangefinder();
     *          if (rfnd != nullptr && rfnd->has_data_orient(ROTATION_PITCH_270)) {
     *              float distance = rfnd->distance_orient(ROTATION_PITCH_270);
     *              // Use downward rangefinder distance
     *          }
     *          @endcode
     *
     * @return Pointer to RangeFinder singleton, nullptr if not instantiated
     *
     * @note Returns nullptr before vehicle initialization completes
     * @note Thread-safe - singleton initialized before scheduler starts
     *
     * Source: libraries/AP_RangeFinder/AP_RangeFinder.h:342
     */
    RangeFinder *rangefinder();
};

#endif  // AP_RANGEFINDER_ENABLED
