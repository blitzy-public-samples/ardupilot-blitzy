/**
 * @file AP_RangeFinder_PulsedLightLRF.h
 * @brief PulsedLight/Garmin LIDAR-Lite I2C laser rangefinder driver
 * 
 * @details This driver implements support for the PulsedLight/Garmin LIDAR-Lite
 *          series of I2C laser rangefinders, which are popular hobby-grade LiDAR
 *          sensors widely used in multicopters and autonomous vehicles.
 *          
 *          Supported Hardware Versions:
 *          - LIDAR-Lite v2 (legacy)
 *          - LIDAR-Lite v3
 *          - LIDAR-Lite v3HP (high performance)
 *          
 *          Key Specifications:
 *          - Interface: I2C (address 0x62)
 *          - Maximum Range: 40 meters
 *          - Accuracy: ±2.5cm at distances <5m
 *          - Update Rate: Up to 500Hz (v3HP), 100Hz typical
 *          - Power: 5V DC, 100mA average
 *          - Field of View: 8mrad
 *          
 *          Communication Protocol:
 *          This driver uses asynchronous I2C communication with a two-phase
 *          measurement cycle:
 *          1. PHASE_MEASURE: Initiates distance acquisition
 *          2. PHASE_COLLECT: Reads distance result from sensor
 *          
 *          The driver automatically detects hardware version by reading
 *          software and hardware version registers on initialization.
 *          
 * @note LIDAR-Lite sensors require good power supply filtering due to
 *       high peak current during laser firing. Poor power can cause
 *       intermittent communication failures.
 *       
 * @warning v2 hardware has known I2C bus locking issues under certain
 *          conditions. v3 and v3HP are recommended for new designs.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

/* Connection diagram
 *
 *        ------------------------------------------------------------------------------------
 *        |           J2-1(LED) J2-2(5V) J2-3(Enable) J2-4(Ref Clk) J2-5(GND) J2-6(GND)      |
 *        |                                                                                  |
 *        |                                                                                  |
 *        |                                      J1-3(I2C Clk) J1-2(I2C Data) J1-1(GND)      |
 *        ------------------------------------------------------------------------------------
 */

/**
 * @class AP_RangeFinder_PulsedLightLRF
 * @brief Driver for PulsedLight/Garmin LIDAR-Lite I2C laser rangefinders
 * 
 * @details This backend implements I2C communication with PulsedLight LIDAR-Lite
 *          rangefinders (also sold as Garmin LIDAR-Lite). These are compact,
 *          affordable laser rangefinders popular in hobby and educational robotics.
 *          
 *          Hardware Detection:
 *          The driver automatically detects and adapts to different hardware
 *          versions (v2, v3, v3HP) by reading version registers on initialization.
 *          Version-specific quirks and timing are handled internally.
 *          
 *          Measurement Cycle:
 *          - PHASE_MEASURE: Write acquisition command (0x04) to register 0x00
 *          - PHASE_COLLECT: Read distance from register 0x8F (2 bytes, big-endian)
 *          - Cycle time: ~10-20ms depending on range and version
 *          
 *          I2C Communication:
 *          - Default address: 0x62 (7-bit)
 *          - Clock speed: 100-400kHz supported
 *          - Bus voltage: 3.3V or 5V compatible
 *          
 *          Distance Calculation:
 *          Raw sensor value represents distance in centimeters directly.
 *          No scaling or calibration required for normal operation.
 *          
 *          Power Management:
 *          Sensor draws significant current during laser firing (up to 150mA).
 *          Ensure adequate power supply decoupling capacitance (680µF recommended).
 *          
 * @note This driver runs asynchronously via scheduler timer callback to avoid
 *       blocking the main loop during I2C transactions.
 *       
 * @warning v2 hardware can lock up I2C bus if power supply is inadequate.
 *          Always use recommended power supply filtering.
 *          
 * @see AP_RangeFinder_Backend for base class interface
 */
class AP_RangeFinder_PulsedLightLRF : public AP_RangeFinder_Backend
{

public:
    /**
     * @brief Detect and initialize PulsedLight LIDAR-Lite sensor on I2C bus
     * 
     * @details This static factory method attempts to detect and initialize
     *          a PulsedLight LIDAR-Lite sensor on the specified I2C bus.
     *          Detection process:
     *          1. Attempt to acquire I2C device at address 0x62
     *          2. Read hardware and software version registers
     *          3. Verify valid version values
     *          4. Configure sensor for continuous operation
     *          5. Register timer callback for asynchronous reading
     *          
     *          The method automatically identifies hardware version (v2/v3/v3HP)
     *          and applies version-specific initialization and timing.
     * 
     * @param[in] bus           I2C bus number (0-3 depending on HAL)
     * @param[in,out] _state    Rangefinder state structure to populate
     * @param[in,out] _params   Rangefinder parameters for this instance
     * @param[in] rftype        Rangefinder type identifier
     * 
     * @return Pointer to new backend instance if detection successful, nullptr if failed
     * 
     * @note Detection can fail if sensor not present, I2C bus error, or
     *       sensor already in use by another driver
     *       
     * @see init() for detailed initialization procedure
     */
    static AP_RangeFinder_Backend *detect(uint8_t bus,
                                          RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          RangeFinder::Type rftype);

    /**
     * @brief Update rangefinder state (main loop interface)
     * 
     * @details This method is called from the main vehicle loop but does not
     *          perform any operations. All sensor reading is handled asynchronously
     *          in the timer() callback to avoid blocking the main loop during
     *          I2C transactions which can take 10-20ms per measurement cycle.
     *          
     *          State updates are applied directly in timer() when new distance
     *          readings are available.
     * 
     * @note This is an empty override of the base class pure virtual method.
     *       Actual sensor updates occur in timer() callback.
     *       
     * @see timer() for actual measurement implementation
     */
    void update(void) override {}

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink enum value identifying this sensor as a
     *          laser-based distance sensor. This information is used in
     *          telemetry messages (DISTANCE_SENSOR) sent to ground control
     *          stations for sensor identification and display.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser rangefinder type
     * 
     * @note This is a laser time-of-flight sensor, not ultrasonic or radar
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @brief Private constructor - instances created via detect() factory method
     * 
     * @details Constructs PulsedLight LIDAR-Lite backend instance with specified
     *          I2C bus and state structures. Constructor initializes base class
     *          and stores parameters but does not perform hardware initialization.
     *          
     *          Hardware initialization is performed in init() which is called
     *          after construction during the detection process.
     * 
     * @param[in] bus       I2C bus number for sensor communication
     * @param[in,out] _state  Rangefinder state structure
     * @param[in,out] _params Rangefinder parameters
     * @param[in] rftype    Rangefinder type identifier
     * 
     * @note Constructor is private - use detect() static method to create instances
     * @see detect() for public instantiation interface
     */
    AP_RangeFinder_PulsedLightLRF(uint8_t bus,
                                  RangeFinder::RangeFinder_State &_state,
								  AP_RangeFinder_Params &_params,
                                  RangeFinder::Type rftype);

    /**
     * @brief Initialize sensor hardware and configure for operation
     * 
     * @details Performs hardware initialization sequence:
     *          1. Read and validate hardware version register (0x41)
     *          2. Read and validate software version register (0x4F)
     *          3. Detect v2 vs v3/v3HP hardware based on version values
     *          4. Apply hardware-specific configuration
     *          5. Configure sensor acquisition mode and timing
     *          6. Verify sensor responds correctly
     *          
     *          Version Detection:
     *          - v2: hw_version <= 0x15
     *          - v3: hw_version > 0x15, not v3HP
     *          - v3HP: Detected via specific version value patterns
     *          
     *          Configuration:
     *          Sets sensor to continuous measurement mode with appropriate
     *          timing parameters for detected hardware version.
     * 
     * @return true if initialization successful, false if sensor not responding
     *              or invalid version detected
     * 
     * @note Called during detect() before backend is registered
     * @warning Initialization failure indicates sensor not present or faulty
     */
    bool init(void);
    
    /**
     * @brief Asynchronous timer callback for sensor reading
     * 
     * @details This method is called periodically by the HAL scheduler (typically
     *          at 50Hz) to manage the asynchronous measurement cycle. It implements
     *          a two-phase state machine:
     *          
     *          PHASE_MEASURE:
     *          - Write acquisition command (0x04) to register 0x00
     *          - Initiates laser pulse and distance measurement
     *          - Transitions to PHASE_COLLECT
     *          - Duration: <1ms (command write)
     *          
     *          PHASE_COLLECT:
     *          - Wait for measurement complete (busy flag clear)
     *          - Read distance from register 0x8F (2 bytes)
     *          - Update state with new distance value
     *          - Transitions back to PHASE_MEASURE
     *          - Duration: 10-20ms (measurement time + read)
     *          
     *          Error Handling:
     *          - I2C communication errors: Increment error counter
     *          - Timeout detection: Reset to PHASE_MEASURE
     *          - Out-of-range values: Mark as unhealthy
     *          
     *          Health Monitoring:
     *          Periodically reads status register to verify sensor health and
     *          detect hardware faults (signal strength, reference oscillator).
     * 
     * @note Runs in scheduler context, not main loop - keep execution time minimal
     * @warning Do not call blocking operations from timer callback
     * 
     * @see PHASE_MEASURE, PHASE_COLLECT for state machine phases
     */
    void timer(void);
    
    /**
     * @brief Perform I2C transfer with sensor
     * 
     * @details Executes I2C write-read transaction with LIDAR-Lite sensor using
     *          HAL I2C device interface. This is a low-level communication
     *          primitive used by timer() to send commands and read results.
     *          
     *          Transaction Flow:
     *          1. Write register address and optional data bytes
     *          2. Repeated start (if receiving data)
     *          3. Read response bytes
     *          
     *          Common Operations:
     *          - Write command: send_len=2 (register + command), recv_len=0
     *          - Read register: send_len=1 (register address), recv_len=1-2
     *          - Write-read: send_len=1-2, recv_len=1-2
     * 
     * @param[in] send      Pointer to data to transmit (register address + data)
     * @param[in] send_len  Number of bytes to transmit (1-N)
     * @param[out] recv     Pointer to buffer for received data (or nullptr)
     * @param[in] recv_len  Number of bytes to receive (0-N)
     * 
     * @return true if I2C transaction successful, false if communication error
     * 
     * @note Uses HAL I2C device transfer() which is non-blocking
     * @warning I2C errors can indicate bus problems or sensor power issues
     */
    bool lidar_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);
    
    /**
     * @brief I2C device handle for sensor communication
     * 
     * @details Smart pointer to HAL I2C device interface. Manages I2C bus access
     *          and provides non-blocking transfer methods. Automatically released
     *          when backend is destroyed, freeing I2C bus for other devices.
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    /**
     * @brief Sensor software version from register 0x4F
     * 
     * @details Software version number reported by sensor firmware. Used to
     *          identify firmware revision and apply version-specific behavior.
     *          Typical values: 0x01-0x0F depending on firmware release.
     */
    uint8_t sw_version;
    
    /**
     * @brief Sensor hardware version from register 0x41
     * 
     * @details Hardware version number used to distinguish between sensor
     *          revisions. Critical for version detection:
     *          - v2 hardware: hw_version <= 0x15 (decimal 21)
     *          - v3/v3HP hardware: hw_version > 0x15
     *          Different hardware versions require different timing and
     *          configuration parameters.
     */
    uint8_t hw_version;
    
    /**
     * @brief Counter for periodic register health checks
     * 
     * @details Incremented on each timer() call and used to trigger periodic
     *          reading of sensor health registers (status, signal strength).
     *          Health checks are performed every N measurement cycles to avoid
     *          excessive I2C traffic while still monitoring sensor condition.
     */
    uint8_t check_reg_counter;
    
    /**
     * @brief Flag indicating v2 hardware detected
     * 
     * @details Set to true if hardware version indicates v2 LIDAR-Lite.
     *          v2 hardware has different timing characteristics and known
     *          I2C reliability issues that require special handling.
     *          
     * @note v2 is legacy hardware; v3/v3HP recommended for new installations
     */
    bool v2_hardware;
    
    /**
     * @brief Flag indicating v3HP (high performance) hardware detected
     * 
     * @details Set to true if sensor is LIDAR-Lite v3HP variant.
     *          v3HP supports higher update rates (up to 500Hz) and has
     *          improved measurement algorithms compared to standard v3.
     *          Enables use of v3HP-specific features and timing.
     */
    bool v3hp_hardware;
    
    /**
     * @brief Most recent distance measurement in centimeters
     * 
     * @details Stores last valid distance reading from sensor for comparison
     *          with new readings. Used for:
     *          - Detecting stuck sensor (repeated identical values)
     *          - Rate limiting updates
     *          - Noise filtering
     *          Range: 0-4000 (0-40 meters)
     */
    uint16_t last_distance_cm;
    
    /**
     * @brief Rangefinder type identifier for this instance
     * 
     * @details Type enum identifying this specific rangefinder configuration.
     *          Used for parameter lookup and multi-rangefinder systems where
     *          different instances may have different roles (downward-facing,
     *          forward-facing, etc.).
     */
    RangeFinder::Type rftype;
    
    /**
     * @enum Measurement cycle state machine phases
     * 
     * @details Two-phase asynchronous measurement cycle:
     *          
     *          PHASE_MEASURE:
     *          - Initiate new distance acquisition
     *          - Write acquisition command to sensor
     *          - Sensor begins laser pulse and timing measurement
     *          - Transition to PHASE_COLLECT
     *          
     *          PHASE_COLLECT:
     *          - Wait for measurement completion
     *          - Read distance result from sensor
     *          - Update rangefinder state with new distance
     *          - Transition back to PHASE_MEASURE for next cycle
     *          
     *          State machine runs in timer() callback at scheduler rate.
     */
    enum { PHASE_MEASURE, PHASE_COLLECT } phase;
};

#endif  // AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
