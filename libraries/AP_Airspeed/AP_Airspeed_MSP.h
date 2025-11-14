/**
 * @file AP_Airspeed_MSP.h
 * @brief MSP (MultiWii Serial Protocol) airspeed sensor backend
 * 
 * @details This backend provides airspeed data integration from MSP-compatible
 *          devices, typically used in racing/FPV vehicle configurations with
 *          OSD systems. The implementation receives airspeed measurements
 *          asynchronously via MSP serial protocol messages, supporting
 *          lightweight telemetry integration without dedicated I2C/SPI buses.
 * 
 *          MSP airspeed messages can provide either:
 *          - Direct differential pressure measurements (Pascals)
 *          - Airspeed velocity (m/s) which is converted to pressure
 *          - Optional temperature data (Celsius)
 * 
 *          Protocol: MSP (MultiWii Serial Protocol)
 *          Interface: Serial (via AP_MSP message accumulator)
 *          Update Rate: Asynchronous, device-dependent (typically 10-50Hz)
 * 
 * @note Vehicle-specific build support controlled by AP_AIRSPEED_MSP_ENABLED
 * @note No hardware probing required - data driven by MSP message reception
 * 
 * @see libraries/AP_MSP/ for MSP protocol implementation
 * @see libraries/AP_Airspeed/AP_Airspeed_Backend.h for base interface
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_MSP_ENABLED

#include "AP_Airspeed_Backend.h"

#include <AP_MSP/msp.h>

/**
 * @class AP_Airspeed_MSP
 * @brief MSP-based airspeed sensor backend for serial protocol integration
 * 
 * @details This backend integrates airspeed measurements received via the
 *          MultiWii Serial Protocol (MSP), commonly used in racing and FPV
 *          applications where airspeed data is provided by OSD systems or
 *          external MSP-compatible devices.
 * 
 *          Key characteristics:
 *          - Asynchronous data reception via MSP message accumulator
 *          - No hardware probing or initialization required
 *          - Averaging of multiple samples for noise reduction
 *          - Thread-safe message handling via base class mechanisms
 *          - Supports both pressure and temperature measurements
 * 
 *          The backend accumulates pressure and temperature samples
 *          received via handle_msp() and returns averaged values when
 *          requested by the airspeed library. Data freshness is tracked
 *          using timestamps to detect stale measurements.
 * 
 *          Typical use case: FPV racing vehicles with MSP OSD integration
 *          where airspeed sensor data is transmitted over serial telemetry
 *          alongside other flight data.
 * 
 * @note This backend does not directly communicate with hardware - it
 *       receives pre-processed measurements via MSP protocol messages
 * 
 * @warning Ensure MSP instance is properly configured and messages are
 *          being received before relying on airspeed data for flight control
 * 
 * @see AP_Airspeed_Backend for base interface
 * @see AP_MSP for MSP protocol implementation
 */
class AP_Airspeed_MSP : public AP_Airspeed_Backend
{
public:
    /**
     * @brief Construct MSP airspeed backend
     * 
     * @param[in] airspeed Reference to parent AP_Airspeed frontend
     * @param[in] instance Airspeed sensor instance number (0-based)
     * @param[in] msp_instance MSP instance for message routing
     * 
     * @details Initializes the backend with specified airspeed instance
     *          and MSP instance for routing incoming messages. The MSP
     *          instance determines which MSP port will provide airspeed data.
     */
    AP_Airspeed_MSP(AP_Airspeed &airspeed, uint8_t instance, uint8_t msp_instance);

    /**
     * @brief Initialize the MSP airspeed backend
     * 
     * @return true Always returns true - no hardware initialization required
     * 
     * @details MSP backend requires no hardware initialization since data
     *          arrives asynchronously via serial protocol messages. The
     *          backend is ready to receive and process MSP airspeed packets
     *          immediately after construction.
     * 
     * @note Unlike I2C/SPI backends, no device probing is performed
     * @note MSP message handlers are registered by the AP_MSP subsystem
     */
    bool init(void) override {
        return true;
    }

    /**
     * @brief Process incoming MSP airspeed data message
     * 
     * @param[in] pkt MSP airspeed data packet containing pressure and/or temperature
     * 
     * @details Called by the AP_MSP subsystem when an MSP airspeed message
     *          is received. Accumulates pressure and temperature samples for
     *          averaging, reducing noise in the measurements. Samples are
     *          accumulated until read by get_differential_pressure() and
     *          get_temperature(), at which point accumulators are reset.
     * 
     *          The packet may contain:
     *          - Differential pressure (Pascals)
     *          - Airspeed velocity (m/s) - converted to pressure internally
     *          - Temperature (Celsius) - optional
     * 
     *          Message reception updates internal timestamp for data freshness
     *          tracking. Stale data detection uses 100ms timeout.
     * 
     * @note Thread-safe - uses base class synchronization mechanisms
     * @note Called at MSP message reception rate (device-dependent, typically 10-50Hz)
     * 
     * @see MSP::msp_airspeed_data_message_t for packet structure
     */
    void handle_msp(const MSP::msp_airspeed_data_message_t &pkt) override;

    /**
     * @brief Get current differential pressure measurement from MSP sensor
     * 
     * @param[out] pressure Differential pressure in Pascals (averaged from accumulated samples)
     * 
     * @return true if fresh data available (received within last 100ms), false if stale
     * 
     * @details Returns averaged differential pressure from all samples accumulated
     *          since last call. Pressure accumulator is reset after each read.
     *          Data freshness is verified using 100ms timeout - if no MSP messages
     *          received within timeout, returns false indicating stale data.
     * 
     *          If MSP device provides airspeed velocity instead of pressure,
     *          conversion to differential pressure is performed using standard
     *          relationship: P = 0.5 * ρ * V² (where ρ is air density)
     * 
     * @note Units: Pascals (Pa)
     * @note Called by airspeed library at main loop rate (typically 10-400Hz)
     * @note Averaging reduces noise from serial transmission and sensor jitter
     * 
     * @see handle_msp() for sample accumulation
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Get current temperature measurement from MSP sensor if available
     * 
     * @param[out] temperature Temperature in degrees Celsius (averaged from accumulated samples)
     * 
     * @return true if temperature data available, false otherwise
     * 
     * @details Returns averaged temperature from all samples accumulated since
     *          last call. Temperature accumulator is reset after each read.
     *          Not all MSP airspeed devices provide temperature - availability
     *          depends on the specific MSP sensor implementation.
     * 
     *          Temperature data is optional in MSP airspeed messages. If the
     *          connected device does not provide temperature, this method
     *          returns false and the output parameter remains unchanged.
     * 
     * @note Units: Degrees Celsius (°C)
     * @note Temperature availability is device-specific
     * @note Used for air density compensation in airspeed calculations
     * 
     * @see handle_msp() for sample accumulation
     */
    bool get_temperature(float &temperature) override;
    
private:
    /**
     * @brief MSP instance identifier for message routing
     * 
     * @details Specifies which MSP serial port instance provides airspeed
     *          data for this backend. Multiple MSP instances can be active
     *          simultaneously on different serial ports.
     */
    const uint8_t msp_instance;
    
    /**
     * @brief Accumulated differential pressure samples in Pascals
     * 
     * @details Sum of all pressure samples received since last read.
     *          Averaged and reset by get_differential_pressure().
     */
    float sum_pressure;
    
    /**
     * @brief Number of accumulated pressure samples
     * 
     * @details Count of pressure samples in sum_pressure accumulator.
     *          Used to calculate average pressure on read.
     */
    uint8_t press_count;
    
    /**
     * @brief Accumulated temperature samples in Celsius
     * 
     * @details Sum of all temperature samples received since last read.
     *          Averaged and reset by get_temperature().
     */
    float sum_temp;
    
    /**
     * @brief Number of accumulated temperature samples
     * 
     * @details Count of temperature samples in sum_temp accumulator.
     *          Used to calculate average temperature on read.
     */
    uint8_t temp_count;
};

#endif // AP_AIRSPEED_MSP_ENABLED
