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
 * @file AP_Airspeed_SDP3X.h
 * @brief Sensirion SDP3x I2C differential pressure sensor backend driver
 * 
 * @details This driver supports the Sensirion SDP3x sensor family (SDP31, SDP32, SDP33)
 *          for differential pressure measurement used in airspeed calculation. The SDP3x
 *          sensors communicate via I2C and provide continuous measurement with built-in
 *          averaging to reduce noise.
 *          
 *          Key features:
 *          - Continuous measurement mode with ~50Hz sampling and averaging
 *          - Sensirion CRC8 validation for data integrity (polynomial 0x31)
 *          - Automatic scale detection based on sensor product ID
 *          - Air density compensation using barometer data for altitude correction
 *          - Temperature compensation for accurate pressure readings
 *          
 *          The driver implements the Sensirion I2C protocol with CRC validation
 *          to ensure data integrity, which is critical for safety in flight control.
 *          
 * @note Register definitions and protocol details credited to PX4 project
 * @see Sensirion SDP3x datasheet for detailed sensor specifications and CRC algorithm
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SDP3X_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"
#include "AP_Airspeed.h"

/**
 * @class AP_Airspeed_SDP3X
 * @brief Backend driver for Sensirion SDP3x differential pressure sensors
 * 
 * @details This class implements the AP_Airspeed_Backend interface for Sensirion
 *          SDP3x series differential pressure sensors (SDP31, SDP32, SDP33). These
 *          sensors measure the pressure difference between pitot and static ports,
 *          which is used to calculate airspeed.
 *          
 *          **Measurement Mode:**
 *          The driver configures the sensor in continuous measurement mode with
 *          averaging, providing stable readings at approximately 50Hz. Samples are
 *          accumulated and averaged to reduce noise before being made available to
 *          the airspeed library.
 *          
 *          **Data Integrity:**
 *          All sensor communications use Sensirion's CRC8 checksum (polynomial 0x31,
 *          initial value 0xFF) to validate data integrity. Invalid CRC results in
 *          sample rejection, ensuring corrupted data never reaches flight control.
 *          
 *          **Scale Detection:**
 *          The driver automatically detects the sensor's pressure scale factor from
 *          the product ID during initialization. Different SDP3x variants have
 *          different pressure ranges and corresponding scale factors.
 *          
 *          **Air Density Compensation:**
 *          Raw pressure readings are compensated for air density using barometer
 *          data. This rho_air correction is essential for accurate airspeed
 *          calculation at varying altitudes, as air density decreases with altitude.
 *          
 *          **Thread Safety:**
 *          Sensor reading occurs in HAL timer callback context. Data is accumulated
 *          with simple counters (no mutex required as access is from single thread).
 *          
 * @note This driver requires an I2C bus and assumes sensor is at standard SDP3x address
 * @warning CRC validation failure indicates corrupted data - reading is discarded for safety
 * @warning Air density compensation is required for accurate airspeed at altitude
 * 
 * @see AP_Airspeed_Backend for base interface
 * @see Sensirion SDP3x datasheet for sensor specifications
 */
class AP_Airspeed_SDP3X : public AP_Airspeed_Backend
{
public:

    /**
     * @brief Constructor using base class constructor
     * 
     * @details Inherits constructor from AP_Airspeed_Backend, initializing
     *          with airspeed frontend reference and sensor instance number.
     */
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    /**
     * @brief Destructor - cleanup I2C device
     * 
     * @details Releases the I2C device handle allocated during initialization.
     *          Called automatically when backend is destroyed.
     */
    ~AP_Airspeed_SDP3X() {
        delete _dev;
    }

    /**
     * @brief Probe and initialize the SDP3x sensor
     * 
     * @details Attempts to detect and configure the SDP3x sensor on the I2C bus:
     *          1. Acquires I2C device handle from HAL
     *          2. Reads sensor product ID to determine scale factor
     *          3. Configures sensor for continuous measurement mode with averaging
     *          4. Registers periodic timer callback for ~50Hz sample reading
     *          
     *          The sensor is configured to provide continuous differential pressure
     *          and temperature readings with built-in averaging to reduce noise.
     *          
     * @return true if sensor successfully probed, configured, and started
     * @return false if sensor not detected or configuration failed
     * 
     * @note Called once during airspeed subsystem initialization
     * @note Failure to initialize means sensor is not available for this flight
     */
    bool init() override;

    /**
     * @brief Return the current density-compensated differential pressure
     * 
     * @details Retrieves the latest averaged differential pressure reading
     *          with air density compensation applied. The raw pressure from
     *          the sensor is corrected using the rho_air (air density) factor
     *          calculated from barometric altitude. This compensation is
     *          essential for accurate airspeed calculation at varying altitudes.
     *          
     *          Data freshness is checked - readings older than 100ms are
     *          considered stale and rejected.
     *          
     * @param[out] pressure Differential pressure in Pascals (Pa)
     * 
     * @return true if fresh pressure data available (< 100ms old)
     * @return false if no data or data is stale
     * 
     * @note Called by airspeed library to get pressure for airspeed calculation
     * @note Air density compensation (rho_air) is critical for altitude accuracy
     * @warning Stale data (>100ms) indicates sensor communication failure
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Return the current sensor temperature
     * 
     * @details Retrieves the latest averaged temperature reading from the
     *          SDP3x sensor's integrated temperature sensor. Temperature
     *          data is used for sensor temperature compensation and is
     *          also available for logging and diagnostics.
     *          
     *          Data freshness is checked - readings older than 100ms are
     *          considered stale and rejected.
     *          
     * @param[out] temperature Sensor temperature in degrees Celsius (°C)
     * 
     * @return true if fresh temperature data available (< 100ms old)
     * @return false if no data or data is stale
     * 
     * @note Called by airspeed library for temperature logging
     * @note Temperature is measured at the sensor location, not ambient air
     */
    bool get_temperature(float &temperature) override;

private:
    /**
     * @brief Periodic timer callback to read sensor data
     * 
     * @details Called at approximately 50Hz by the HAL scheduler to read
     *          differential pressure and temperature from the SDP3x sensor.
     *          Each successful reading is accumulated into sum accumulators
     *          with corresponding count increments for averaging.
     *          
     *          The callback performs:
     *          1. I2C transfer to read sensor data frame
     *          2. CRC8 validation of received data
     *          3. Conversion from raw ADC values to physical units
     *          4. Accumulation for averaging
     *          5. Timestamp update for freshness checking
     *          
     *          If CRC validation fails, the sample is discarded and error
     *          count incremented. This ensures corrupted data never affects
     *          flight control calculations.
     *          
     * @note Runs in HAL timer callback context (interrupt priority)
     * @note Sample accumulation is thread-safe (single-threaded access)
     * @warning Failed CRC indicates I2C communication error or sensor fault
     */
    void _timer();
    
    /**
     * @brief Send command to SDP3x sensor via I2C
     * 
     * @details Transmits a 16-bit command to the sensor using the Sensirion
     *          I2C protocol. Commands are used to configure measurement mode,
     *          read product ID, and control sensor operation. The 16-bit
     *          command is sent MSB first as per Sensirion protocol.
     *          
     * @param[in] cmd 16-bit command code (see SDP3x datasheet for command list)
     * 
     * @return true if I2C transfer successful
     * @return false if I2C communication error
     * 
     * @note Common commands: start continuous measurement, read product ID, soft reset
     */
    bool _send_command(uint16_t cmd);
    
    /**
     * @brief Validate Sensirion CRC8 checksum
     * 
     * @details Computes Sensirion CRC8 checksum over data and compares with
     *          expected checksum value. Sensirion uses CRC8 with polynomial
     *          0x31 (x^8 + x^5 + x^4 + 1) and initial value 0xFF.
     *          
     *          The SDP3x protocol includes a CRC byte after every 2 data bytes
     *          to detect transmission errors. This validation is critical for
     *          safety-critical airspeed measurements.
     *          
     * @param[in] data Pointer to data bytes to validate
     * @param[in] size Number of data bytes (not including CRC byte)
     * @param[in] checksum Expected CRC8 value from sensor
     * 
     * @return true if computed CRC matches expected checksum
     * @return false if CRC mismatch indicates corrupted data
     * 
     * @note Polynomial 0x31, initial value 0xFF per Sensirion specification
     * @warning CRC failure must result in data rejection for safety
     * @see Sensirion application note for CRC8 algorithm details
     */
    bool _crc(const uint8_t data[], uint8_t size, uint8_t checksum);
    
    /**
     * @brief Apply air density correction to raw pressure reading
     * 
     * @details Compensates differential pressure for air density variation
     *          with altitude. The correction uses the rho_air factor obtained
     *          from the barometer to adjust pressure for current altitude.
     *          
     *          Air density decreases with altitude, affecting the relationship
     *          between differential pressure and true airspeed. Without this
     *          correction, indicated airspeed would be inaccurate at altitude.
     *          
     *          Correction formula applies density ratio to maintain calibrated
     *          airspeed accuracy across altitude range.
     *          
     * @param[in] press Raw differential pressure from sensor in Pascals
     * 
     * @return Density-corrected differential pressure in Pascals
     * 
     * @note Uses barometric altitude data via AP_Baro for density calculation
     * @warning Altitude compensation is essential for accurate airspeed at altitude
     * @see get_differential_pressure() where corrected pressure is returned
     */
    float _correct_pressure(float press);

    /**
     * @brief Last valid temperature reading in degrees Celsius
     * 
     * @details Updated when accumulated temperature samples are averaged
     *          and made available to the frontend. Value represents sensor
     *          temperature, not ambient air temperature.
     *          
     * Units: degrees Celsius (°C)
     */
    float _temp;
    
    /**
     * @brief Last valid density-corrected pressure reading in Pascals
     * 
     * @details Updated when accumulated pressure samples are averaged,
     *          density corrected, and made available to the frontend.
     *          This is the final differential pressure value used for
     *          airspeed calculation.
     *          
     * Units: Pascals (Pa)
     */
    float _press;
    
    /**
     * @brief Count of accumulated temperature samples
     * 
     * @details Incremented for each valid temperature reading received
     *          in _timer() callback. Used as divisor when computing
     *          average temperature. Reset to zero after averaging.
     *          
     * Units: sample count (dimensionless)
     */
    uint16_t _temp_count;
    
    /**
     * @brief Count of accumulated pressure samples
     * 
     * @details Incremented for each valid pressure reading received
     *          in _timer() callback. Used as divisor when computing
     *          average pressure. Reset to zero after averaging.
     *          
     * Units: sample count (dimensionless)
     */
    uint16_t _press_count;
    
    /**
     * @brief Accumulator for temperature samples
     * 
     * @details Running sum of temperature readings from sensor. Each
     *          valid sample from _timer() is added to this accumulator.
     *          Averaged by dividing by _temp_count when frontend requests
     *          temperature data, then reset to zero.
     *          
     * Units: degrees Celsius (°C) - sum of samples
     */
    float _temp_sum;
    
    /**
     * @brief Accumulator for pressure samples
     * 
     * @details Running sum of pressure readings from sensor. Each valid
     *          sample from _timer() is added to this accumulator.
     *          Averaged by dividing by _press_count when frontend requests
     *          pressure data, then reset to zero.
     *          
     * Units: Pascals (Pa) - sum of samples
     */
    float _press_sum;
    
    /**
     * @brief Timestamp of last successful sensor reading
     * 
     * @details Updated to current system time (AP_HAL::millis()) whenever
     *          a valid sensor reading with correct CRC is received in the
     *          _timer() callback. Used to detect stale data - readings
     *          older than 100ms are considered stale and rejected.
     *          
     *          Stale data indicates sensor communication failure or sensor
     *          malfunction requiring operator attention.
     *          
     * Units: milliseconds (ms) - system time since boot
     * 
     * @note 100ms timeout allows for several missed readings before failure
     * @warning Stale data must not be used for flight control
     */
    uint32_t _last_sample_time_ms;
    
    /**
     * @brief Pressure scale factor for raw ADC to Pascal conversion
     * 
     * @details Scale factor determined during initialization by reading
     *          the sensor product ID. Different SDP3x variants (SDP31,
     *          SDP32, SDP33) have different pressure ranges and require
     *          different scale factors to convert raw ADC values to Pascals.
     *          
     *          The scale factor represents Pascals per raw ADC unit.
     *          Typical values depend on sensor variant and measurement range.
     *          
     * Units: Pascals per ADC count (Pa/count)
     * 
     * @note Read from sensor during init() and remains constant during operation
     * @see init() for scale factor detection logic
     */
    uint16_t _scale;

    /**
     * @brief HAL I2C device interface pointer
     * 
     * @details Pointer to the I2C device handle allocated during init().
     *          Used for all I2C communication with the SDP3x sensor.
     *          The handle encapsulates I2C bus, device address, transfer
     *          speed, and provides thread-safe I2C operations.
     *          
     *          Device is acquired from AP_HAL::I2CDeviceManager during
     *          initialization and released in destructor.
     *          
     * @note Ownership of device handle is transferred to this class
     * @see init() for device acquisition
     * @see ~AP_Airspeed_SDP3X() for device cleanup
     */
    AP_HAL::I2CDevice *_dev;
};


#endif  // AP_AIRSPEED_SDP3X_ENABLED
