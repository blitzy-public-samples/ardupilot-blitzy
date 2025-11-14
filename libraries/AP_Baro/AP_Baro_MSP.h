/**
 * @file AP_Baro_MSP.h
 * @brief MSP (MultiWii Serial Protocol) barometer backend driver
 * 
 * @details This driver implements barometer support for external devices
 *          using the MSP protocol. It receives barometric pressure and
 *          temperature data via MSP messages from external flight controllers,
 *          OSD modules, or other MSP-compatible devices.
 *          
 *          The driver applies a moving average filter to smooth incoming
 *          sensor readings and integrates them into ArduPilot's barometer
 *          subsystem for altitude estimation and navigation.
 * 
 * @note MSP protocol is commonly used by Betaflight, iNav, and other
 *       flight controller systems for telemetry and sensor data exchange.
 * 
 * @see AP_Baro_Backend.h for the barometer backend interface
 * @see libraries/AP_MSP/ for MSP protocol implementation details
 */
#pragma once

#include "AP_Baro_Backend.h"

// AP_BARO_MSP_ENABLED is defined in AP_Baro.h

#if AP_BARO_MSP_ENABLED

/**
 * @brief Moving average filter weight for pressure/temperature smoothing
 * 
 * @details Weight factor of 0.20 creates a 5-sample moving average filter
 *          to reduce noise in MSP barometer readings. Lower values provide
 *          more smoothing but increase lag; higher values are more responsive
 *          but noisier.
 */
#define MOVING_AVERAGE_WEIGHT 0.20f // a 5 samples moving average

/**
 * @class AP_Baro_MSP
 * @brief Barometer backend for receiving barometric data via MSP protocol
 * 
 * @details This class implements a barometer backend that consumes barometric
 *          pressure and temperature measurements from external devices using
 *          the MultiWii Serial Protocol (MSP). Instead of directly interfacing
 *          with a physical barometer sensor, this backend receives pre-processed
 *          sensor data through MSP messages.
 *          
 *          The driver is designed for systems where:
 *          - An external flight controller provides barometer data via MSP
 *          - An OSD module with integrated barometer sends telemetry
 *          - Multiple flight control systems share sensor data
 *          
 *          Key features:
 *          - Moving average filtering for noise reduction
 *          - Accumulation-based averaging for stable readings
 *          - Support for multiple MSP instances
 *          - Integration with ArduPilot EKF and navigation systems
 *          
 *          Data flow: External Device → MSP Message → handle_msp() → 
 *                     Moving Average → update() → ArduPilot Baro Subsystem
 * 
 * @note This backend is enabled by AP_BARO_MSP_ENABLED build flag
 * @note Multiple MSP barometer instances can coexist for sensor redundancy
 * 
 * @warning Ensure MSP message rate is sufficient (>10Hz recommended) to
 *          prevent stale barometer data from affecting altitude estimation
 */
class AP_Baro_MSP : public AP_Baro_Backend
{
public:
    /**
     * @brief Constructor for MSP barometer backend
     * 
     * @details Initializes the MSP barometer backend and registers it with
     *          the main barometer driver. Creates an instance that listens
     *          for MSP barometer messages from a specific MSP instance.
     * 
     * @param[in] baro Reference to the main AP_Baro driver for registration
     * @param[in] msp_instance MSP instance ID to associate with (0-based index)
     *                         for systems with multiple MSP connections
     * 
     * @note The instance will accumulate pressure/temperature data until
     *       update() is called by the main scheduler
     */
    AP_Baro_MSP(AP_Baro &baro, uint8_t msp_instance);
    
    /**
     * @brief Update barometer state with accumulated MSP data
     * 
     * @details Called periodically by the barometer scheduler to push accumulated
     *          pressure and temperature readings to the barometer frontend.
     *          Applies the moving average filter and resets accumulation counters.
     *          
     *          This method is called at the barometer update rate (typically 10-50Hz)
     *          and provides filtered data to ArduPilot's EKF for altitude estimation.
     * 
     * @note Overrides AP_Baro_Backend::update()
     * @note If no data has been accumulated since last update, previous values
     *       are maintained to prevent altitude jumps
     * 
     * @see handle_msp() for where raw MSP data is initially received
     */
    void update(void) override;
    
    /**
     * @brief Process incoming MSP barometer data packet
     * 
     * @details Callback function invoked when an MSP barometer message is received
     *          from the external device. Accumulates pressure and temperature values
     *          for averaging in the next update() cycle.
     *          
     *          MSP baro packet format (from external flight controller):
     *          - Pressure: Atmospheric pressure in Pascals (Pa)
     *          - Temperature: Ambient temperature in degrees Celsius (°C)
     *          
     *          The method maintains running sums (sum_pressure, sum_temp) and a
     *          sample count, which update() uses to calculate filtered averages.
     * 
     * @param[in] pkt MSP barometer data message containing:
     *                - pkt.altitude: Calculated altitude from external device (not used)
     *                - pkt.pressure: Barometric pressure in Pascals
     *                - pkt.temperature: Temperature in Celsius
     * 
     * @note Overrides AP_Baro_Backend::handle_msp()
     * @note This is called asynchronously when MSP data arrives, potentially at
     *       rates different from the main barometer update loop
     * @note Units from MSP: Pressure in Pa, Temperature in °C
     * @note Allows using barometer data from external MSP-compatible devices
     *       such as OSD modules, external flight controllers, or sensor hubs
     * 
     * @see update() for where accumulated data is processed and published
     */
    void handle_msp(const MSP::msp_baro_data_message_t &pkt) override;

private:
    /**
     * @brief Barometer instance ID registered with AP_Baro frontend
     * 
     * @details Instance number assigned by the main barometer driver, used
     *          when publishing pressure/temperature updates to the frontend.
     *          Allows multiple barometer backends to coexist in the system.
     */
    uint8_t instance;
    
    /**
     * @brief MSP instance ID for protocol routing
     * 
     * @details Identifies which MSP communication channel this barometer
     *          backend is associated with. Supports systems with multiple
     *          MSP connections (e.g., separate OSD and telemetry links).
     */
    uint8_t msp_instance;
    
    /**
     * @brief Accumulated pressure sum for moving average calculation
     * 
     * @details Running sum of pressure readings in Pascals (Pa) received
     *          via MSP messages since the last update() call. Divided by
     *          count to produce the averaged pressure value.
     *          
     *          Units: Pascals (Pa)
     */
    float sum_pressure;
    
    /**
     * @brief Accumulated temperature sum for moving average calculation
     * 
     * @details Running sum of temperature readings in degrees Celsius (°C)
     *          received via MSP messages since the last update() call.
     *          Divided by count to produce the averaged temperature value.
     *          
     *          Units: Degrees Celsius (°C)
     */
    float sum_temp;
    
    /**
     * @brief Number of samples accumulated since last update
     * 
     * @details Count of MSP barometer packets received via handle_msp()
     *          since the last update() cycle. Used as divisor for computing
     *          average pressure and temperature values.
     *          
     *          Reset to zero after each update() call.
     */
    uint16_t count;
};

#endif // AP_BARO_MSP_ENABLED
