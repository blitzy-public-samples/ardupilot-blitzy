/**
 * @file AP_InertialSensor_ExternalAHRS.h
 * @brief External AHRS IMU data adapter backend
 * 
 * @details Implements backend adapter for IMU data provided by external AHRS systems
 *          via MAVLink or custom protocols, presenting it as a native IMU backend.
 *          This allows high-end navigation-grade IMUs and external sensor fusion
 *          systems to integrate seamlessly with ArduPilot's inertial sensor framework.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#if AP_EXTERNAL_AHRS_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_ExternalAHRS
 * @brief Backend adapter for external AHRS IMU data
 * 
 * @details Adapter layer allowing external AHRS systems to provide IMU data:
 *          - Receives IMU packets from external AHRS systems
 *          - Translates external data to AP_InertialSensor internal format
 *          - Registers as regular IMU backend for seamless integration
 *          - Supports multiple external AHRS sources simultaneously
 *          
 *          Supported external AHRS systems:
 *          - VectorNav VN-100/200/300 series
 *          - Lord MicroStrain 3DM-GX5/CV5 series
 *          - Custom MAVLink IMU data sources
 *          - Other systems via AP_ExternalAHRS framework
 *          
 *          Data flow architecture:
 *          1. External AHRS sends IMU data via serial/CAN/Ethernet
 *          2. AP_ExternalAHRS parses protocol-specific packets
 *          3. Parsed data passed to handle_external() method
 *          4. Driver publishes to frontend as native IMU samples
 *          5. EKF and attitude controllers consume as normal IMU
 *          
 *          Primary use cases:
 *          - High-end navigation-grade IMUs with superior performance
 *          - Pre-filtered/fused IMU data from external processors
 *          - Offloading sensor fusion to dedicated hardware
 *          - Integration with existing AHRS infrastructure
 *          - Testing with simulation or logged IMU data sources
 *          
 *          Thread safety:
 *          - handle_external() called from protocol parser thread
 *          - update() called from main scheduler loop
 *          - Synchronization handled by frontend accumulator
 * 
 * @note Requires AP_EXTERNAL_AHRS_ENABLED compile flag to be set
 * @note External IMU bypasses standard calibration - assumes pre-calibrated data
 * @warning Latency and jitter from external source affects control loop performance
 * @warning Ensure external AHRS coordinate frame matches ArduPilot conventions (NED)
 * 
 * @see AP_ExternalAHRS for protocol parsing and configuration
 * @see AP_InertialSensor_Backend for base backend interface
 */
class AP_InertialSensor_ExternalAHRS : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Construct External AHRS IMU backend instance
     * 
     * @details Initializes backend adapter for receiving IMU data from external AHRS.
     *          Registers with frontend and prepares to receive data via handle_external().
     *          Does not start receiving data until start() is called by frontend.
     * 
     * @param[in] imu          Reference to frontend AP_InertialSensor instance
     * @param[in] serial_port  Index of external AHRS providing this IMU data (0-based)
     * 
     * @note Constructor does not allocate gyro/accel instances - done in start()
     * @note Multiple instances can exist for multi-AHRS redundancy
     */
    AP_InertialSensor_ExternalAHRS(AP_InertialSensor &imu, uint8_t serial_port);

    /**
     * @brief Update IMU state and publish samples to frontend
     * 
     * @details Called by frontend at main scheduler rate to process accumulated samples.
     *          Publishes gyro and accel data received via handle_external() to frontend.
     *          Implements standard backend update protocol for sample publication.
     *          
     *          Update sequence:
     *          1. Check if samples available from external source
     *          2. Apply any backend-specific corrections
     *          3. Publish to frontend via _publish_gyro() and _publish_accel()
     *          4. Update sample timestamps and counters
     * 
     * @return true if new samples were published, false if no new data available
     * 
     * @note Called at main loop rate (typically 400Hz for copter, 50Hz for plane)
     * @note External source may provide data at different rate - handled by accumulator
     * @warning Returning false indicates no new data - may trigger IMU health warnings
     * 
     * @see AP_InertialSensor_Backend::update() for base update protocol
     */
    bool update() override;
    
    /**
     * @brief Start the IMU backend and allocate sensor instances
     * 
     * @details Called once during initialization to start receiving IMU data.
     *          Allocates gyro and accel instances in frontend and marks backend as active.
     *          After start(), handle_external() will begin processing incoming data.
     *          
     *          Initialization sequence:
     *          1. Allocate gyro instance via _imu.register_gyro()
     *          2. Allocate accel instance via _imu.register_accel()
     *          3. Set started flag to enable data processing
     *          4. Configure sample rates based on external source
     * 
     * @note Called once during AP_InertialSensor::_detect_backends()
     * @note Backend must be started before handle_external() will process data
     * @warning Failure to start prevents all IMU data from this source
     * 
     * @see AP_InertialSensor_Backend::start() for base start protocol
     */
    void start() override;
    
    /**
     * @brief Accumulate sensor samples (not used for external AHRS)
     * 
     * @details External AHRS backend does not use the standard accumulation mechanism
     *          since samples arrive asynchronously via handle_external() rather than
     *          being read from hardware at scheduler rate. This method is required by
     *          backend interface but performs no operation for external sources.
     * 
     * @note Standard backends use accumulate() for high-rate sensor sampling
     * @note External AHRS samples accumulated directly in handle_external()
     * 
     * @see handle_external() for actual sample accumulation mechanism
     */
    void accumulate() override;

    /**
     * @brief Handle incoming IMU data packet from external AHRS
     * 
     * @details Called by AP_ExternalAHRS when IMU data packet received from external source.
     *          Processes and accumulates gyro/accel samples for publication in update().
     *          This is the primary data ingestion point for external IMU data.
     *          
     *          Processing sequence:
     *          1. Verify backend is started and ready
     *          2. Extract gyro and accel vectors from packet
     *          3. Apply coordinate frame transformations if needed
     *          4. Accumulate samples using frontend accumulator
     *          5. Update timestamps and sample counters
     *          
     *          Packet contents:
     *          - 3-axis gyroscope data (rad/s in body frame)
     *          - 3-axis accelerometer data (m/s² in body frame)
     *          - Sample timestamp (microseconds)
     *          - Optional temperature data
     * 
     * @param[in] pkt  IMU data message from external AHRS system
     * 
     * @note Called from AP_ExternalAHRS protocol parser (different thread context)
     * @note Sample rate determined by external AHRS configuration
     * @warning Coordinate frame must match ArduPilot conventions (NED, right-hand)
     * @warning High latency or jitter degrades control performance
     * 
     * @see AP_ExternalAHRS::ins_data_message_t for packet structure
     * @see AP_InertialSensor_Backend::handle_external() for base interface
     */
    void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt) override;
    
    /**
     * @brief Get human-readable identification banner for this IMU
     * 
     * @details Generates identification string for display during initialization
     *          and diagnostics. Shows IMU type and associated external AHRS port.
     *          Used in startup banner and IMU sensor listings.
     * 
     * @param[out] banner      Buffer to store identification string
     * @param[in]  banner_len  Maximum length of banner buffer including null terminator
     * 
     * @return true if banner generated successfully, false if buffer too small
     * 
     * @note Typical output: "ExternalAHRS[0]" or "ExtAHRS[1]"
     * @note Called during initialization and for diagnostic displays
     * 
     * @see AP_InertialSensor_Backend::get_output_banner() for base interface
     */
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    /**
     * @brief Index of external AHRS providing this IMU data
     * 
     * @details Identifies which AP_ExternalAHRS instance is providing data to this backend.
     *          Used for routing incoming packets and generating diagnostic information.
     *          Allows multiple external AHRS sources to coexist with independent backends.
     * 
     * @note Zero-based index into AP_ExternalAHRS configured instances
     * @note Set during construction and never modified (const)
     */
    const uint8_t serial_port;
    
    /**
     * @brief Backend initialization state flag
     * 
     * @details Tracks whether start() has been called and backend is ready to process data.
     *          Prevents handle_external() from processing data before gyro/accel instances
     *          are allocated. Set to true by start() during backend initialization.
     * 
     * @note Initially false until start() called by frontend
     * @note handle_external() checks this flag before processing packets
     */
    bool started;
};
#endif // AP_EXTERNAL_AHRS_ENABLED

