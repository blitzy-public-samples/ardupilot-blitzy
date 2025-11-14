/**
 * @file AP_Baro_ExternalAHRS.h
 * @brief Barometer driver for external AHRS systems
 * 
 * @details This file implements a barometer backend that receives barometric
 *          pressure and temperature data from external Attitude and Heading
 *          Reference Systems (AHRS). This allows ArduPilot to use barometer
 *          data from integrated navigation systems such as VectorNav, LORD
 *          Microstrain, and other external AHRS units that provide barometric
 *          altitude information alongside attitude data.
 *          
 *          The driver accumulates multiple readings and provides averaged
 *          measurements to the main barometer library for improved accuracy
 *          and noise reduction.
 * 
 * @note This backend is only used when an external AHRS system is configured
 *       and that system provides integrated barometer readings.
 */
#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_EXTERNALAHRS_ENABLED

/**
 * @class AP_Baro_ExternalAHRS
 * @brief Barometer backend for external AHRS systems that provide barometric data
 * 
 * @details This class integrates barometer data from external AHRS systems into
 *          ArduPilot's barometer subsystem. External AHRS devices like VectorNav
 *          VN-100/200/300, LORD Microstrain 3DM-GX5/GQ7, and similar navigation
 *          systems often include integrated barometric pressure sensors alongside
 *          their IMU and magnetometer sensors.
 *          
 *          Rather than requiring a separate barometer sensor, this backend allows
 *          ArduPilot to use the barometer data provided by the external AHRS unit
 *          through the ExternalAHRS protocol. This reduces sensor redundancy and
 *          can improve data synchronization between attitude and altitude measurements.
 *          
 *          The implementation accumulates multiple barometer readings received from
 *          the external system and averages them during each update() cycle to
 *          reduce noise and provide smooth altitude estimates.
 * 
 * @note Only active when AP_BARO_EXTERNALAHRS_ENABLED is set and an external
 *       AHRS device is configured with barometer support enabled.
 * 
 * @see AP_ExternalAHRS for the external AHRS interface protocol
 * @see AP_Baro_Backend for the base barometer backend interface
 */
class AP_Baro_ExternalAHRS : public AP_Baro_Backend
{
public:
    /**
     * @brief Construct an external AHRS barometer backend
     * 
     * @details Initializes the barometer backend to receive barometric data from
     *          an external AHRS system. Registers this barometer instance with the
     *          AP_Baro library and prepares to accumulate readings from the external
     *          navigation system.
     * 
     * @param[in] baro        Reference to the main AP_Baro object for registration
     * @param[in] serial_port Serial port number connected to the external AHRS device
     * 
     * @note This constructor is typically called during external AHRS initialization
     *       when the external system reports barometer capability.
     */
    AP_Baro_ExternalAHRS(AP_Baro &baro, uint8_t serial_port);
    
    /**
     * @brief Update barometer readings with accumulated external AHRS data
     * 
     * @details Called periodically by the main barometer library to update the
     *          barometer state. This method averages all barometric readings
     *          accumulated since the last update (via handle_external() calls)
     *          and publishes the averaged pressure and temperature to the
     *          barometer library.
     *          
     *          If no new readings have been received since the last update, this
     *          method will not update the barometer state, allowing the library
     *          to detect stale data or connection loss to the external AHRS.
     * 
     * @note Called at the barometer update rate (typically 20-100 Hz)
     * 
     * @see handle_external() for the accumulation of individual readings
     */
    void update(void) override;
    
    /**
     * @brief Process incoming barometer data from external AHRS system
     * 
     * @details This callback is invoked by the ExternalAHRS library whenever a new
     *          barometer data message is received from the external navigation system.
     *          The method accumulates the pressure and temperature readings for
     *          averaging during the next update() call.
     *          
     *          Multiple readings may be accumulated between update() calls if the
     *          external AHRS provides barometer data at a higher rate than the
     *          ArduPilot barometer update rate. This accumulation provides noise
     *          reduction through averaging.
     *          
     *          Units received from external system:
     *          - Pressure: Pascals (Pa)
     *          - Temperature: Degrees Celsius (°C)
     * 
     * @param[in] pkt Barometer data message from external AHRS containing pressure
     *                and temperature readings in standard units
     * 
     * @note This method is called from the ExternalAHRS message processing context,
     *       which may be a different thread or task than the main barometer update.
     *       Thread safety is handled through the accumulation pattern.
     * 
     * @warning Do not call this method directly - it is invoked by the ExternalAHRS
     *          protocol handler when barometer messages are received.
     * 
     * @see AP_ExternalAHRS::baro_data_message_t for the message structure definition
     */
    void handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt) override;

private:
    uint8_t instance;        ///< Barometer instance number registered with AP_Baro library
    float sum_pressure;      ///< Accumulated pressure sum in Pascals for averaging
    float sum_temp;          ///< Accumulated temperature sum in Celsius for averaging
    uint16_t count;          ///< Number of readings accumulated since last update()
};

#endif // AP_BARO_EXTERNALAHRS_ENABLED
