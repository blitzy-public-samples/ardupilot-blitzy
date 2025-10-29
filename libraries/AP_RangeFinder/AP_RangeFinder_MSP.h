/**
 * @file AP_RangeFinder_MSP.h
 * @brief MSP (MultiWii Serial Protocol) rangefinder backend implementation
 * 
 * @details This file implements a rangefinder backend that receives distance
 *          data via the MSP protocol. MSP is commonly used for communication
 *          with OSD (On-Screen Display) devices and flight controllers that
 *          support the MultiWii/Betaflight/iNav protocols.
 *          
 *          The backend receives distance measurements through MSP messages
 *          and provides them to the ArduPilot rangefinder subsystem. This
 *          allows integration of MSP-compatible distance sensors, typically
 *          used in conjunction with MSP OSD systems.
 *          
 *          Architecture:
 *          - Inherits from AP_RangeFinder_Backend base class
 *          - Receives distance data via handle_msp() callback
 *          - Implements timeout-based data validation
 *          - No active polling - purely message-driven
 * 
 * @see AP_RangeFinder_Backend
 * @see AP_MSP (MSP protocol implementation)
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if HAL_MSP_RANGEFINDER_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

/**
 * @brief Data timeout for MSP rangefinder measurements
 * 
 * @details Maximum time (in milliseconds) allowed between MSP distance
 *          messages before the rangefinder is considered unhealthy.
 *          If no data is received within this timeout period, the
 *          sensor state will be marked as out-of-range or no-data.
 *          
 *          Value of 500ms balances responsiveness with tolerance for
 *          typical MSP communication delays and occasional packet loss.
 */
#define AP_RANGEFINDER_MSP_TIMEOUT_MS 500

/**
 * @class AP_RangeFinder_MSP
 * @brief MSP protocol rangefinder backend
 * 
 * @details This class implements a rangefinder backend that receives distance
 *          measurements via the MSP (MultiWii Serial Protocol). Unlike active
 *          rangefinder backends that poll sensors directly, this backend is
 *          passive and relies on external MSP messages to provide distance data.
 *          
 *          Typical usage scenario:
 *          - MSP OSD device receives distance from connected sensor
 *          - OSD forwards distance data via MSP protocol to ArduPilot
 *          - This backend receives data through handle_msp() callback
 *          - Distance is validated and provided to rangefinder subsystem
 *          
 *          Integration with AP_MSP:
 *          The AP_MSP library handles MSP protocol parsing and message routing.
 *          When a rangefinder message is received, AP_MSP calls handle_msp()
 *          on this backend to deliver the distance measurement.
 *          
 *          Thread Safety:
 *          - handle_msp() called from MSP receive context
 *          - update() called from main scheduler thread
 *          - Distance data protected by backend state locking
 *          
 *          Hardware Dependencies:
 *          - Requires MSP-enabled serial port (configured via AP_MSP)
 *          - Compatible with MSP OSD devices supporting rangefinder messages
 *          - No direct hardware access (sensor managed by external device)
 * 
 * @note This backend does not actively poll hardware - it only processes
 *       incoming MSP messages containing distance data.
 * 
 * @warning Ensure MSP serial port is properly configured and MSP OSD device
 *          is sending rangefinder messages at sufficient rate (>2Hz recommended)
 *          to avoid timeout-induced sensor failures.
 */
class AP_RangeFinder_MSP : public AP_RangeFinder_Backend
{

public:
    /**
     * @brief Constructor for MSP rangefinder backend
     * 
     * @details Initializes the MSP rangefinder backend with state and parameter
     *          references. The constructor sets up the backend but does not
     *          initiate any communication - the backend is passive and waits
     *          for incoming MSP messages.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for this instance
     * @param[in] _params Reference to rangefinder parameters for this instance
     * 
     * @note Constructor is called by AP_RangeFinder when MSP rangefinder type is configured
     * @see AP_RangeFinder_Backend::AP_RangeFinder_Backend()
     */
    AP_RangeFinder_MSP(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Static detection function for MSP rangefinder availability
     * 
     * @details Checks whether MSP rangefinder support is available in the system.
     *          For MSP rangefinders, detection does not probe physical hardware
     *          but rather verifies that the MSP protocol subsystem is available
     *          and configured.
     *          
     *          This function is called during rangefinder initialization when
     *          RNGFND_TYPE is set to MSP. Unlike hardware-probing backends,
     *          MSP rangefinder can be detected without physical sensor presence
     *          since the sensor is managed by an external MSP device.
     * 
     * @return true if MSP rangefinder backend can be used, false otherwise
     * 
     * @note This always returns true when HAL_MSP_RANGEFINDER_ENABLED is defined
     * @see AP_RangeFinder::detect_instance()
     */
    static bool detect();

    /**
     * @brief Update rangefinder state based on received MSP data
     * 
     * @details Called periodically by the rangefinder scheduler to update sensor
     *          state. This method checks data freshness using the timeout value
     *          and updates the sensor health status accordingly.
     *          
     *          Algorithm:
     *          1. Check time since last MSP message received
     *          2. If timeout exceeded, mark sensor as no-data/unhealthy
     *          3. If data is fresh, state remains valid (updated by handle_msp)
     *          
     *          Unlike active rangefinder backends, this does not initiate new
     *          measurements - it only validates existing data freshness.
     * 
     * @note Called at scheduler rate (typically 20Hz)
     * @warning If MSP messages stop arriving, sensor will timeout after
     *          AP_RANGEFINDER_MSP_TIMEOUT_MS milliseconds
     * 
     * @see handle_msp() - where actual distance data is received
     * @see AP_RANGEFINDER_MSP_TIMEOUT_MS
     */
    void update(void) override;

    /**
     * @brief Receive and process distance data from MSP protocol message
     * 
     * @details This is the primary data input method for the MSP rangefinder backend.
     *          Called by the AP_MSP library when a rangefinder data message is
     *          received via the MSP protocol. The method extracts distance information
     *          from the MSP packet and updates the backend state.
     *          
     *          MSP Protocol Integration:
     *          - MSP message type: MSP_RANGEFINDER (or similar - see AP_MSP)
     *          - Message contains distance measurement in centimeters
     *          - Called from MSP receive thread/interrupt context
     *          - Must complete quickly to avoid blocking MSP processing
     *          
     *          Data Processing:
     *          1. Extract distance from MSP packet structure
     *          2. Validate distance is within sensor min/max range
     *          3. Update internal distance_cm storage
     *          4. Update last-received timestamp
     *          5. Update sensor state (healthy, in-range, etc.)
     *          
     *          OSD Integration Context:
     *          MSP rangefinder typically used with OSD devices that:
     *          - Receive distance from local sensor (ultrasonic, lidar, etc.)
     *          - Package distance into MSP protocol message
     *          - Transmit to ArduPilot over configured MSP serial port
     *          - This method receives and processes that transmitted data
     *          
     *          Thread Safety:
     *          - May be called from MSP serial receive context
     *          - Must be thread-safe with respect to update() method
     *          - Backend state locking handled by base class
     * 
     * @param[in] pkt MSP rangefinder data message packet containing distance measurement
     *                Structure defined in AP_MSP library (see MSP::msp_rangefinder_data_message_t)
     * 
     * @note This is a callback method invoked by AP_MSP - not called directly by user code
     * @warning Do not perform blocking operations in this method as it may be called
     *          from interrupt context or high-priority MSP processing thread
     * 
     * @see AP_MSP - MSP protocol implementation and message routing
     * @see update() - periodic state validation
     */
    void handle_msp(const MSP::msp_rangefinder_data_message_t &pkt) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type for this rangefinder
     * 
     * @details Returns the MAVLink distance sensor type enumeration value
     *          that best describes this rangefinder's sensing technology.
     *          This value is reported in MAVLink DISTANCE_SENSOR messages
     *          to ground control stations.
     *          
     *          For MSP rangefinders, the actual sensor type is unknown because
     *          the physical sensor is managed by an external MSP device (OSD).
     *          The MSP protocol does not communicate sensor type information,
     *          only the distance measurement itself.
     *          
     *          MAV_DISTANCE_SENSOR_UNKNOWN indicates to ground stations that
     *          the specific sensing technology (ultrasonic, laser, infrared, etc.)
     *          cannot be determined from the available information.
     * 
     * @return MAV_DISTANCE_SENSOR_UNKNOWN - sensor type cannot be determined
     *         via MSP protocol
     * 
     * @note Physical sensor type depends on what is connected to the MSP OSD device
     * @see MAV_DISTANCE_SENSOR enum in MAVLink common.xml
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    /**
     * @brief Cached distance measurement in centimeters
     * 
     * @details Stores the most recent distance measurement received via
     *          handle_msp() from MSP protocol. Value is in centimeters
     *          matching the standard ArduPilot rangefinder unit convention.
     *          
     *          This value is updated by handle_msp() when new MSP distance
     *          messages arrive and is used by update() to provide distance
     *          to the rangefinder subsystem.
     *          
     *          Valid range depends on configured RNGFND_MIN_CM and RNGFND_MAX_CM
     *          parameters, but storage supports 0-65535 cm (0-655.35 meters).
     * 
     * @note Updated from MSP receive context, read from scheduler context
     */
    uint16_t distance_cm;

    /**
     * @brief Initiate a new distance reading (not used for MSP rangefinder)
     * 
     * @details This static method is part of the rangefinder backend interface
     *          for sensors that require active polling to initiate measurements.
     *          For MSP rangefinders, readings are not actively initiated by
     *          ArduPilot but are instead pushed via MSP messages from external
     *          devices.
     *          
     *          This method exists for interface compatibility but should not
     *          be called for MSP rangefinders. MSP is a passive, message-driven
     *          backend that does not control sensor timing.
     * 
     * @return bool - Success/failure status (implementation specific)
     * 
     * @note MSP rangefinders do not actively poll - this method not used
     * @see handle_msp() - where MSP rangefinder actually receives data
     */
    static bool start_reading(void);
    
    /**
     * @brief Retrieve a distance reading (not used for MSP rangefinder)
     * 
     * @details This static method is part of the rangefinder backend interface
     *          for sensors where reading initiation (start_reading) and data
     *          retrieval (get_reading) are separate operations.
     *          
     *          For MSP rangefinders, distance data arrives asynchronously via
     *          MSP messages and is processed in handle_msp(). There is no
     *          separate "get reading" operation since data is pushed rather
     *          than pulled.
     *          
     *          This method exists for interface compatibility but should not
     *          be called for MSP rangefinders.
     * 
     * @param[out] reading_cm Distance reading in centimeters (if successful)
     * 
     * @return bool - Success/failure status (implementation specific)
     * 
     * @note MSP rangefinders receive data via handle_msp() - this method not used
     * @see handle_msp() - where MSP rangefinder actually receives data
     */
    static bool get_reading(uint16_t &reading_cm);
};

#endif  //HAL_MSP_RANGEFINDER_ENABLED

