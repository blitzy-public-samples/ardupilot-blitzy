/**
 * @file AP_Airspeed_External.h
 * @brief External AHRS airspeed message backend
 * 
 * This file implements an airspeed sensor backend that receives airspeed data
 * from external AHRS/INS systems rather than directly from a physical sensor.
 * This is used when an integrated navigation system (such as VectorNav, Lord
 * Microstrain, or similar) includes an airspeed sensor and provides airspeed
 * measurements through the AP_ExternalAHRS interface.
 * 
 * The backend accumulates incoming airspeed data messages and provides averaged
 * differential pressure and temperature readings when requested by the frontend.
 * 
 * @see AP_ExternalAHRS for the external AHRS interface
 * @see AP_Airspeed_Backend for the base airspeed backend interface
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_EXTERNAL_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

/**
 * @class AP_Airspeed_External
 * @brief Airspeed backend that receives data from external AHRS systems
 * 
 * @details This backend implements an airspeed sensor that receives measurements
 * from external AHRS/INS systems through the AP_ExternalAHRS interface rather
 * than directly reading from a physical sensor via I2C or analog input.
 * 
 * Use Case: Integrated navigation systems with built-in airspeed sensors
 * - VectorNav INS with pitot-static sensor
 * - Lord Microstrain systems with airspeed input
 * - Other external AHRS systems providing airspeed data
 * 
 * Architecture: Message-driven aggregator
 * - Receives asynchronous airspeed_data_message_t from external AHRS
 * - Accumulates differential pressure and temperature values
 * - Averages accumulated values when frontend requests readings
 * - Implements overflow protection when accumulation exceeds 100 samples
 * 
 * Advantages:
 * - Consolidated sensor suite with factory calibration
 * - Reduced wiring and integration complexity
 * - Leverages external system's sensor processing
 * - No I2C bus contention or timing requirements
 * 
 * Thread Safety: All data access protected by semaphore for safe
 * multi-threaded operation between message handler and frontend reads.
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_External.h
 *         libraries/AP_Airspeed/AP_Airspeed_External.cpp
 */
class AP_Airspeed_External : public AP_Airspeed_Backend
{
public:
    /**
     * @brief Construct an external AHRS airspeed backend
     * 
     * @param[in] airspeed Reference to the frontend AP_Airspeed object
     * @param[in] instance Instance number for this airspeed sensor (0-based)
     * 
     * @details Initializes the backend using the base class constructor and
     * sets up a virtual bus ID for the external serial interface. No physical
     * sensor probing is required since data comes from external AHRS messages.
     */
    AP_Airspeed_External(AP_Airspeed &airspeed, uint8_t instance);

    /**
     * @brief Initialize the external airspeed backend
     * 
     * @return true always (no hardware probing required)
     * 
     * @details Unlike physical sensor backends, external AHRS airspeed requires
     * no initialization or probing sequence. The backend simply waits for
     * airspeed_data_message_t packets from the external AHRS system. Always
     * returns true to indicate the backend is ready to receive messages.
     * 
     * @note No I2C/SPI bus access or sensor detection is performed
     */
    bool init(void) override {
        return true;
    }

    /**
     * @brief Process incoming airspeed data from external AHRS system
     * 
     * @param[in] pkt Airspeed data message containing differential pressure and temperature
     * 
     * @details This method is called asynchronously when the external AHRS system
     * sends airspeed data. The method accumulates the differential pressure (Pascal)
     * and temperature (Celsius) values for later averaging. Implements overflow
     * protection by halving accumulated sums and counts when they exceed 100 samples.
     * 
     * Thread Safety: Protected by semaphore for safe concurrent access with
     * get_differential_pressure() and get_temperature() calls from frontend.
     * 
     * @note Called from AP_ExternalAHRS context, not main thread
     * @see AP_ExternalAHRS::airspeed_data_message_t for message structure
     */
    void handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt) override;

    /**
     * @brief Get averaged differential pressure from external AHRS
     * 
     * @param[out] pressure Averaged differential pressure in Pascal
     * 
     * @return true if data available (at least one sample since last read)
     * @return false if no data available since last read
     * 
     * @details Returns the average of all differential pressure samples received
     * since the last call to this method. After computing the average, the
     * accumulator and count are reset to zero for the next averaging period.
     * 
     * Thread Safety: Protected by semaphore for safe concurrent access with
     * handle_external() message processing.
     * 
     * Units: Output pressure in Pascal (Pa)
     * 
     * @note Resets accumulator after each successful read
     * @warning Returns false if no new data received - frontend should handle stale data
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Get averaged temperature from external AHRS
     * 
     * @param[out] temperature Averaged temperature in Celsius
     * 
     * @return true if data available (at least one sample since last read)
     * @return false if no data available since last read
     * 
     * @details Returns the average of all temperature samples received since
     * the last call to this method. After computing the average, the accumulator
     * and count are reset to zero for the next averaging period. Temperature
     * is provided by the external AHRS system along with differential pressure.
     * 
     * Thread Safety: Protected by semaphore for safe concurrent access with
     * handle_external() message processing.
     * 
     * Units: Output temperature in degrees Celsius (°C)
     * 
     * @note Resets accumulator after each successful read
     * @warning Returns false if no new data received - frontend should handle stale data
     */
    bool get_temperature(float &temperature) override;
    
private:
    /**
     * @brief Accumulated differential pressure sum for averaging
     * 
     * Accumulates differential pressure values (Pascal) from incoming
     * airspeed_data_message_t packets. Reset to zero after each
     * get_differential_pressure() call. Protected by semaphore.
     * 
     * Units: Pascal (Pa)
     */
    float sum_pressure;
    
    /**
     * @brief Number of differential pressure samples accumulated
     * 
     * Counts the number of pressure samples in sum_pressure. Reset to zero
     * after each get_differential_pressure() call. Used to compute average.
     * Implements overflow protection by halving when count exceeds 100.
     * Protected by semaphore.
     */
    uint8_t press_count;
    
    /**
     * @brief Accumulated temperature sum for averaging
     * 
     * Accumulates temperature values (Celsius) from incoming
     * airspeed_data_message_t packets. Reset to zero after each
     * get_temperature() call. Protected by semaphore.
     * 
     * Units: degrees Celsius (°C)
     */
    float sum_temperature;
    
    /**
     * @brief Number of temperature samples accumulated
     * 
     * Counts the number of temperature samples in sum_temperature. Reset to
     * zero after each get_temperature() call. Used to compute average.
     * Implements overflow protection by halving when count exceeds 100.
     * Protected by semaphore.
     */
    uint8_t temperature_count;
};

#endif // AP_AIRSPEED_EXTERNAL_ENABLED

