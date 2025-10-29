/**
 * @file AP_OpticalFlow_HereFlow.h
 * @brief HereFlow DroneCAN/UAVCAN optical flow sensor backend
 * 
 * @details This backend implements support for the Hex/ProfiCNC HereFlow optical flow sensor,
 *          which communicates via DroneCAN (UAVCAN) protocol over CAN bus. The sensor performs
 *          onboard optical flow processing and transmits integrated measurements.
 * 
 * Protocol: com.hex.equipment.flow.Measurement DroneCAN messages
 * Hardware: Hex/ProfiCNC HereFlow optical flow sensor with onboard processing
 * Communication: CAN bus interface via AP_DroneCAN
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_HEREFLOW_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

/**
 * @class AP_OpticalFlow_HereFlow
 * @brief DroneCAN optical flow sensor backend for HereFlow
 * 
 * @details This backend subscribes to DroneCAN flow measurement messages from the HereFlow sensor.
 *          Key features and behavior:
 *          - Subscribes to com.hex.equipment.flow.Measurement DroneCAN messages
 *          - Enforces single-instance per node ID (only one HereFlow sensor per CAN bus)
 *          - Converts integral measurements (pixel*seconds) to flow rates (rad/s)
 *          - Accumulates gyro integrals (rad*seconds) for velocity compensation
 *          - Supports quality metric (0-255) and source node filtering
 * 
 *          Data Flow:
 *          1. DroneCAN message arrives → handle_measurement() callback (CAN thread)
 *          2. Integral data stored with semaphore protection
 *          3. update() called by scheduler → _push_state() converts integrals to rates
 *          4. Rates passed to frontend via _update_frontend()
 * 
 * @note Single instance enforcement - only one HereFlow per CAN bus supported
 * @warning Requires AP_DroneCAN enabled and HAL_ENABLE_DRONECAN_DRIVERS compile flag
 * 
 * @see OpticalFlow_backend
 * @see AP_DroneCAN
 */
class AP_OpticalFlow_HereFlow : public OpticalFlow_backend {
public:
    /**
     * @brief Initialize HereFlow backend
     * 
     * @param[in] flow Reference to AP_OpticalFlow manager for sensor registration
     * 
     * @note Constructor does not perform hardware initialization; actual subscription
     *       to DroneCAN messages happens in subscribe_msgs()
     */
    AP_OpticalFlow_HereFlow(AP_OpticalFlow &flow);

    /**
     * @brief No-op initialization for DroneCAN backend
     * 
     * @details Subscription to DroneCAN messages happens in subscribe_msgs() which is
     *          called by the DroneCAN subsystem, not through the standard init() path.
     *          This override exists to satisfy the backend interface requirement.
     * 
     * @note Override of OpticalFlow_backend::init()
     */
    void init() override {}

    /**
     * @brief Transfer accumulated data from CAN callback to frontend
     * 
     * @details Called periodically by the scheduler. If new_data flag is set, calls
     *          _push_state() to convert accumulated integral measurements to flow rates
     *          and pass them to the frontend via _update_frontend(). Resets new_data flag
     *          after processing.
     * 
     * @note Called at scheduler rate (typically 50-400Hz depending on vehicle type)
     * @note Override of OpticalFlow_backend::update()
     * 
     * @see _push_state()
     */
    void update() override;

    /**
     * @brief Static subscription setup for DroneCAN flow messages
     * 
     * @details Registers a callback to receive com.hex.equipment.flow.Measurement messages
     *          from the DroneCAN bus. Creates singleton driver instance if not already created.
     *          This is called by AP_DroneCAN during initialization to set up message subscriptions.
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance for message subscription
     * 
     * @return true if subscription successful, false otherwise
     * 
     * @note Must be called before any HereFlow messages can be received
     * @warning Only one HereFlow instance supported per CAN bus
     * 
     * @see handle_measurement()
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    /**
     * @brief Static DroneCAN message callback for flow measurements
     * 
     * @details Receives com.hex.equipment.flow.Measurement messages from DroneCAN bus and stores
     *          the integral flow and gyro data. This callback runs in the CAN thread context,
     *          so data storage is protected with a semaphore. The accumulated data is later
     *          processed by update() running in the scheduler thread.
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance (used for semaphore access)
     * @param[in] transfer    Canard RX transfer metadata (contains source node ID, timing)
     * @param[in] msg         Decoded com_hex_equipment_flow_Measurement message containing:
     *                        - integration_interval: Time period for integrals (seconds)
     *                        - flow_integral: Accumulated pixel motion (pixels*seconds)
     *                        - rate_gyro_integral: Accumulated gyro data (rad*seconds)
     *                        - quality: Quality metric 0-255
     * 
     * @note Runs in CAN thread - uses semaphore protection for thread safety
     * @note Filters by node ID if _node_id is non-zero
     * @warning Must not perform blocking operations - stores data and returns quickly
     * 
     * @see update()
     */
    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_hex_equipment_flow_Measurement &msg);

private:

    /// Accumulated pixel motion from CAN message (pixels*seconds), converted to flow rates (rad/s) in sensor body frame
    Vector2f flow_integral;
    
    /// Accumulated gyro measurements for velocity compensation (rad*seconds), converted to rad/s
    Vector2f rate_gyro_integral;
    
    /// Quality metric from sensor, range 0-255 where higher values indicate better quality
    uint8_t surface_quality;
    
    /// Integration period for accumulated measurements (seconds), used to convert integrals to rates
    float integral_time;
    
    /// Flag indicating fresh data available from CAN callback, reset after processing in update()
    bool new_data;
    
    /// Static CAN node ID filter: 0 = accept from all nodes, non-zero = accept only from specific node ID
    static uint8_t _node_id;

    /// Static singleton instance pointer for CAN callback routing
    static AP_OpticalFlow_HereFlow* _driver;
    
    /// Static DroneCAN interface pointer for accessing semaphore and CAN operations
    static AP_DroneCAN* _ap_dronecan;
    
    /**
     * @brief Convert accumulated integrals to rates and update frontend
     * 
     * @details Converts flow_integral and rate_gyro_integral from accumulated values (pixels*seconds, rad*seconds)
     *          to instantaneous rates (rad/s) by dividing by integral_time. Validates that dt is positive
     *          before conversion. Calls _update_frontend() with converted rates, quality, and timestamp.
     * 
     * @note Called from update() when new_data flag is set
     * @warning Requires valid integral_time > 0 to avoid division by zero
     * 
     * @see update()
     * @see _update_frontend()
     */
    void _push_state(void);

};

#endif  // AP_OPTICALFLOW_HEREFLOW_ENABLED
