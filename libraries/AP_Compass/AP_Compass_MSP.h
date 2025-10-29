/**
 * @file AP_Compass_MSP.h
 * @brief MSP (MultiWii Serial Protocol) compass backend driver
 * 
 * This driver receives compass/magnetometer data via MSP packets from external
 * sources such as OSD modules or external flight controllers. MSP is commonly
 * used for integrating sensors from MultiWii/Betaflight/iNav-compatible devices.
 * 
 * @note Applications include OSD integration and external flight controller sensor sharing
 * @see AP_MSP library for MSP protocol implementation details
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MSP_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_MSP/msp.h>

/**
 * @class AP_Compass_MSP
 * @brief Compass backend that receives magnetometer data via MSP protocol packets
 * 
 * @details This backend processes compass data transmitted via the MultiWii Serial Protocol (MSP).
 *          Unlike traditional compass drivers that directly read I2C/SPI sensors, this driver
 *          receives pre-processed magnetometer readings through MSP packets from external devices.
 *          
 *          The driver registers with the MSP subsystem to receive MSP_COMPASS messages containing
 *          3-axis magnetometer readings. Data is provided in milligauss units and aligned with
 *          ArduPilot's NED (North-East-Down) coordinate frame conventions.
 *          
 *          Typical use cases:
 *          - Receiving compass data from an OSD module with integrated magnetometer
 *          - Sharing sensor data from an external MultiWii/Betaflight flight controller
 *          - Integration with MSP-compatible peripheral devices
 *          
 *          Coordinate Frame: NED (North-East-Down) body frame
 *          Units: milligauss (mGauss) for all magnetic field components
 * 
 * @note This is a packet-based driver - data arrives asynchronously via MSP messages
 * @warning Ensure external device coordinate frame alignment matches ArduPilot conventions
 */
class AP_Compass_MSP : public AP_Compass_Backend
{
public:
    /**
     * @brief Construct MSP compass backend for specified MSP instance
     * 
     * @param[in] msp_instance MSP serial port instance number (0-based)
     * 
     * @details Initializes the compass backend and registers with the MSP subsystem
     *          to receive compass data packets on the specified MSP instance.
     *          The constructor allocates a compass instance through the AP_Compass
     *          frontend and prepares for asynchronous packet reception.
     */
    AP_Compass_MSP(uint8_t msp_instance);

    /**
     * @brief Read compass data (pass-through implementation)
     * 
     * @details This method is called by the AP_Compass frontend at regular intervals
     *          but performs no active sensor reading. MSP compass data arrives
     *          asynchronously via handle_msp() when packets are received, so this
     *          method serves as a no-op pass-through to satisfy the backend interface.
     *          
     *          The actual compass data updates occur in handle_msp() when
     *          MSP_COMPASS messages arrive from the external device.
     * 
     * @note Data updates happen asynchronously in handle_msp(), not in read()
     * @see handle_msp() for actual data processing
     */
    void read(void) override;

private:
    /**
     * @brief Process incoming MSP compass data packet
     * 
     * @param[in] pkt MSP compass message containing 3-axis magnetometer readings
     * 
     * @details Handles MSP_COMPASS messages received from external devices via the MSP protocol.
     *          The packet contains magnetic field measurements in milligauss for all three axes.
     *          
     *          Message Format (MSP_COMPASS):
     *          - Field X: Magnetic field in body-frame X axis (milligauss)
     *          - Field Y: Magnetic field in body-frame Y axis (milligauss)
     *          - Field Z: Magnetic field in body-frame Z axis (milligauss)
     *          
     *          Processing Steps:
     *          1. Extract magnetic field components from packet
     *          2. Verify coordinate frame alignment (NED body frame expected)
     *          3. Publish data to AP_Compass frontend via publish_field()
     *          4. Update timestamp for data freshness tracking
     *          
     *          Coordinate System: Data expected in ArduPilot NED body frame
     *          - X: Forward (North)
     *          - Y: Right (East)
     *          - Z: Down
     *          
     *          Units: milligauss (mGauss)
     * 
     * @note Called asynchronously when MSP packets arrive, not at fixed rate
     * @warning External device must provide data in correct coordinate frame
     * @see MSP::msp_compass_data_message_t for packet structure definition
     */
    void handle_msp(const MSP::msp_compass_data_message_t &pkt) override;
    
    /**
     * @brief MSP serial port instance number for this compass
     * 
     * @details Identifies which MSP serial interface this compass backend monitors.
     *          ArduPilot supports multiple MSP ports, and this field specifies which
     *          port instance provides compass data for this backend.
     */
    uint8_t msp_instance;
    
    /**
     * @brief AP_Compass frontend instance ID for this backend
     * 
     * @details Compass instance identifier allocated by AP_Compass frontend during
     *          initialization. Used when publishing sensor readings to ensure data
     *          is associated with the correct compass instance in multi-compass systems.
     */
    uint8_t instance;
};

#endif // AP_COMPASS_MSP_ENABLED
