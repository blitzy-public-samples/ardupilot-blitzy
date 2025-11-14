/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_Airspeed_DLVR.h
 * @brief Backend driver for AllSensors DLVR differential pressure sensor
 * 
 * @details This driver supports the AllSensors DLVR (Digital Low Voltage Range)
 *          series of differential pressure sensors used for airspeed measurement.
 *          
 *          Supported pressure ranges (in inches H2O): 5, 10, 20, 30, 60
 *          
 *          The DLVR sensor communicates via I2C and provides a 4-byte data frame:
 *          - Byte 0: Status bits [7:6] power status, stale data detection
 *          - Bytes 1-3: 24-bit pressure reading (MSB first)
 *          - 11-bit temperature reading embedded in pressure data
 *          
 *          Update rate: 50Hz capable
 *          
 *          Reference: AllSensors DLVR Series Datasheet for data frame format
 *                     and conversion formulas
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_DLVR.h
 *         libraries/AP_Airspeed/AP_Airspeed_DLVR.cpp
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_DLVR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

/**
 * @class AP_Airspeed_DLVR
 * @brief Backend driver for AllSensors DLVR differential pressure sensors
 * 
 * @details This class implements support for AllSensors DLVR series differential
 *          pressure sensors used for airspeed measurement in aircraft applications.
 *          
 *          Key Features:
 *          - Multiple pressure range support via PSI_RANGE parameter (5-60 inH2O)
 *          - 4-byte I2C data frame: status + 24-bit pressure + 11-bit temperature
 *          - 50Hz update rate capability
 *          - Transient temperature guarding and averaging strategy
 *          - Automatic stale data detection via status bits
 *          
 *          Data Frame Format (4 bytes):
 *          - Byte 0 [7:6]: Power status bits (00=normal, 01=warming up, 10=error)
 *          - Byte 0 [5:0]: Most significant bits of pressure (bits 23-18)
 *          - Byte 1: Pressure bits 17-10
 *          - Byte 2: Pressure bits 9-2
 *          - Byte 3 [7:6]: Pressure bits 1-0
 *          - Byte 3 [5:0]: Temperature bits 10-5 (remaining bits in pressure bytes)
 *          
 *          Pressure Scaling:
 *          raw_value → percentage of full scale → PSI → Pascals
 *          Full scale = 80% to 100% of configured range
 *          
 *          Temperature Filtering:
 *          Transient temperature spikes are rejected to maintain sensor stability.
 *          Both pressure and temperature values are averaged over multiple samples
 *          to reduce noise.
 *          
 * @note Status bits must be checked on every read - stale data indicates sensor
 *       communication issues or power problems
 * 
 * @warning Incorrect pressure range configuration will result in inaccurate
 *          airspeed readings that could affect flight safety
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_DLVR.cpp for conversion implementation
 */
class AP_Airspeed_DLVR : public AP_Airspeed_Backend
{
public:

    /**
     * @brief Constructor for DLVR airspeed sensor backend
     * 
     * @param[in] frontend Reference to the AP_Airspeed frontend managing this sensor
     * @param[in] _instance Sensor instance number for multi-sensor configurations
     * @param[in] _range_inH2O Pressure range selection in inches of water (5, 10, 20, 30, or 60)
     * 
     * @details Initializes the DLVR backend with the specified pressure range. The range
     *          parameter determines the full-scale pressure measurement capability and
     *          affects the conversion from raw sensor values to Pascals.
     *          
     *          Common ranges:
     *          - 5 inH2O: Low-speed aircraft (typical small UAVs)
     *          - 10-20 inH2O: Medium-speed aircraft
     *          - 30-60 inH2O: High-speed aircraft
     */
    AP_Airspeed_DLVR(AP_Airspeed &frontend, uint8_t _instance, const float _range_inH2O);
    
    /**
     * @brief Probe and detect DLVR sensor on I2C bus
     * 
     * @param[in] frontend Reference to the AP_Airspeed frontend
     * @param[in] _instance Sensor instance number
     * @param[in] _dev I2C device interface pointer for communication
     * @param[in] _range_inH2O Pressure range in inches of water
     * 
     * @return Pointer to newly created AP_Airspeed_DLVR backend if sensor detected,
     *         nullptr if sensor not found or communication failed
     * 
     * @details Attempts to communicate with the DLVR sensor on the I2C bus to verify
     *          presence and functionality. If successful, creates and returns a new
     *          backend instance. The I2C device ownership is transferred to the backend.
     */
    static AP_Airspeed_Backend *probe(AP_Airspeed &frontend, uint8_t _instance, AP_HAL::I2CDevice *_dev, const float _range_inH2O);

    /**
     * @brief Destructor for DLVR airspeed sensor backend
     * 
     * @details Cleans up I2C device interface and releases allocated resources.
     *          Ensures proper shutdown of sensor communication.
     */
    ~AP_Airspeed_DLVR(void) {
        delete dev;
    }

    /**
     * @brief Probe and initialize the DLVR sensor
     * 
     * @return true if sensor successfully initialized and timer registered, false on failure
     * 
     * @details Initializes the I2C communication with the DLVR sensor and registers
     *          a periodic timer callback for data collection at approximately 50Hz.
     *          Verifies sensor presence by attempting initial communication.
     *          
     *          Initialization sequence:
     *          1. Configure I2C bus parameters
     *          2. Attempt initial sensor read to verify presence
     *          3. Register periodic timer callback for continuous data collection
     *          4. Initialize averaging accumulators
     * 
     * @note This method must be called before attempting to read pressure or temperature
     * @note Timer callback frequency determines maximum sensor update rate
     */
    bool init() override;

    /**
     * @brief Get the current differential pressure reading
     * 
     * @param[out] pressure Differential pressure in Pascals (Pa)
     * 
     * @return true if fresh data available (updated within last 100ms), false if stale
     * 
     * @details Returns the averaged differential pressure measurement. The pressure
     *          value is accumulated from multiple sensor readings and averaged to
     *          reduce noise. Data is considered stale if no updates received within
     *          the last 100 milliseconds.
     *          
     *          Pressure conversion chain:
     *          raw_value (24-bit) → percentage of full scale → PSI → Pascals
     *          
     *          Typical pressure range: 0 to configured range_inH2O
     *          Units: Pascals (Pa)
     * 
     * @note Pressure value is only updated when status bits indicate valid data
     * @note 100ms timeout ensures detection of sensor communication failures
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Get the current sensor temperature reading
     * 
     * @param[out] temperature Sensor temperature in degrees Celsius (°C)
     * 
     * @return true if temperature data available, false if unavailable
     * 
     * @details Returns the averaged sensor temperature measurement extracted from
     *          the 11-bit temperature field in the DLVR data frame. Temperature
     *          values are averaged over multiple readings to reduce noise, and
     *          transient temperature spikes are filtered to maintain stability.
     *          
     *          Temperature is used for:
     *          - Pressure compensation in the sensor
     *          - System health monitoring
     *          - Environmental condition logging
     *          
     *          Units: Degrees Celsius (°C)
     * 
     * @note Transient temperature filter rejects sudden spikes that may indicate
     *       electrical noise or sensor malfunction
     */
    bool get_temperature(float &temperature) override;

private:
    /**
     * @brief Periodic timer callback for sensor data collection
     * 
     * @details Called at approximately 50Hz by the HAL scheduler. Orchestrates
     *          the sensor data collection and averaging process by calling the
     *          appropriate internal methods.
     *          
     *          Timer sequence:
     *          1. Read 4-byte data frame from sensor via I2C
     *          2. Parse status bits and validate data freshness
     *          3. Extract and convert pressure reading
     *          4. Extract and convert temperature reading
     *          5. Apply transient filters and update averages
     *          6. Update timestamp for freshness checking
     * 
     * @note This method runs in interrupt context - keep processing minimal
     * @note 50Hz rate provides good noise averaging while maintaining responsiveness
     */
    void timer();

    /**
     * @brief Last valid pressure reading in Pascals (Pa)
     * 
     * @details Stores the most recent averaged differential pressure measurement
     *          returned by get_differential_pressure(). Updated when averaging
     *          accumulator is processed.
     */
    float pressure;
    
    /**
     * @brief Last valid temperature reading in degrees Celsius (°C)
     * 
     * @details Stores the most recent averaged temperature measurement returned
     *          by get_temperature(). Updated when averaging accumulator is processed.
     */
    float temperature;
    
    /**
     * @brief Temperature averaging accumulator in degrees Celsius (°C)
     * 
     * @details Accumulates temperature readings for noise reduction. Sum is divided
     *          by temp_count to produce averaged temperature value. Reset after
     *          average is computed.
     */
    float temperature_sum;
    
    /**
     * @brief Pressure averaging accumulator in Pascals (Pa)
     * 
     * @details Accumulates pressure readings for noise reduction. Sum is divided
     *          by press_count to produce averaged pressure value. Reset after
     *          average is computed.
     */
    float pressure_sum;
    
    /**
     * @brief Temperature sample counter
     * 
     * @details Counts number of valid temperature samples in temperature_sum
     *          accumulator. Used to compute average temperature. Reset after
     *          average is computed.
     */
    uint32_t temp_count;
    
    /**
     * @brief Pressure sample counter
     * 
     * @details Counts number of valid pressure samples in pressure_sum accumulator.
     *          Used to compute average pressure. Reset after average is computed.
     */
    uint32_t press_count;
    
    /**
     * @brief Timestamp of last valid sensor reading in milliseconds
     * 
     * @details Updated each time fresh data is received from the sensor. Used
     *          to detect stale data - readings older than 100ms are considered
     *          invalid, indicating communication failure or sensor malfunction.
     *          
     *          Units: milliseconds (ms)
     */
    uint32_t last_sample_time_ms;
    
    /**
     * @brief Configured pressure range in inches of water (inH2O)
     * 
     * @details Stores the full-scale pressure range used for converting raw
     *          24-bit sensor readings to physical pressure values. Typical values:
     *          5, 10, 20, 30, or 60 inH2O. This value is set at construction and
     *          determines the pressure scaling factor.
     *          
     *          Units: inches of water (inH2O)
     */
    const float range_inH2O;

    /**
     * @brief Initialize and configure the sensor
     * 
     * @details Performs initial sensor setup including I2C communication parameter
     *          configuration and verification of sensor responsiveness. Called
     *          internally during initialization sequence.
     */
    void setup();

    /**
     * @brief HAL I2C device interface pointer
     * 
     * @details Provides I2C communication interface to the DLVR sensor hardware.
     *          Ownership is managed by this class - allocated during probe(),
     *          deallocated in destructor.
     */
    AP_HAL::I2CDevice *dev;
};

#endif  // AP_AIRSPEED_DLVR_ENABLED
