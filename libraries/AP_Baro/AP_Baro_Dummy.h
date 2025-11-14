/**
 * @file AP_Baro_Dummy.h
 * @brief Dummy barometer driver for testing and hardware bringup
 * 
 * @details This driver provides a non-functional barometer backend that returns
 *          fixed static pressure and temperature values. It is primarily used for:
 *          - Hardware board bringup when no physical sensor is connected
 *          - Software testing without requiring actual hardware
 *          - SITL simulation scenarios where sensor data is provided externally
 *          - Validating system architecture before sensor integration
 * 
 *          The dummy driver is selected using the BARO line in hwdef.dat during
 *          board configuration.
 * 
 * @warning This driver is NOT intended for production use or actual flight operations.
 *          It provides no real atmospheric pressure measurements.
 * 
 * @note Useful for initial board bringup, software testing, and simulation environments
 *       without requiring physical barometer hardware.
 */
#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_DUMMY_ENABLED

/**
 * @class AP_Baro_Dummy
 * @brief Dummy barometer backend that returns static fixed values
 * 
 * @details The Dummy backend implements the AP_Baro_Backend interface but provides
 *          only static pressure and temperature readings rather than actual sensor data.
 *          This allows the system to function during board bringup and testing phases
 *          when physical sensors are not available or not yet configured.
 * 
 *          The driver returns fixed values for:
 *          - Atmospheric pressure (static default value)
 *          - Temperature (static default value)
 *          - Does not provide real altitude or pressure variation measurements
 * 
 * @note This backend is automatically instantiated when specified in board hwdef.dat
 *       configuration files during hardware bring-up phases.
 * 
 * @warning Not suitable for actual flight operations - provides no real sensor data
 */
class AP_Baro_Dummy : public AP_Baro_Backend
{
public:
    /**
     * @brief Constructor for dummy barometer backend
     * 
     * @param[in] baro Reference to the main AP_Baro object managing all barometer instances
     * 
     * @details Initializes the dummy barometer backend and registers it with the main
     *          barometer manager. Sets up fixed default values for pressure and temperature
     *          that will be returned by subsequent update() calls.
     */
    AP_Baro_Dummy(AP_Baro &baro);
    
    /**
     * @brief Update barometer readings with fixed static values
     * 
     * @details This method is called periodically by the scheduler to update sensor readings.
     *          For the dummy driver, this always reports the same fixed pressure and temperature
     *          values rather than reading from actual hardware.
     * 
     * @note Returns static/fixed values - does not provide real atmospheric measurements
     * @warning Do not use for actual flight - provides no altitude or pressure variation data
     */
    void update(void) override;
    
    /**
     * @brief Factory method to detect and instantiate dummy barometer backend
     * 
     * @param[in] baro Reference to the main AP_Baro object
     * @return Pointer to new AP_Baro_Dummy instance, or nullptr if allocation fails
     * 
     * @details This static method is called during sensor probing/detection phase.
     *          For the dummy driver, it always succeeds (no hardware detection required)
     *          and creates a new dummy backend instance.
     * 
     * @note Unlike real sensor drivers, this probe always succeeds since no hardware
     *       detection is performed.
     */
    static AP_Baro_Backend *probe(AP_Baro &baro) {
        return NEW_NOTHROW AP_Baro_Dummy(baro);
    }

private:
    uint8_t _instance;  ///< Instance number for this barometer in the AP_Baro sensor array
};

#endif  // AP_BARO_DUMMY_ENABLED
