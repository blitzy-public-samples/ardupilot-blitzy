/**
 * @file AP_OpticalFlow_MAV.h
 * @brief MAVLink OPTICAL_FLOW message backend for optical flow sensors
 * 
 * @details This backend receives optical flow measurements via MAVLink
 *          OPTICAL_FLOW messages (message ID 100) from external sources such as:
 *          - Companion computers running optical flow processing
 *          - External optical flow processors with MAVLink interface
 *          - Offboard flow computation modules
 *          
 *          Protocol: MAVLink OPTICAL_FLOW message (message ID 100)
 *          Communication: Any MAVLink telemetry link (Serial, UDP, TCP)
 *          
 *          The backend accumulates flow measurements between update() calls,
 *          averages quality metrics, and applies gyro compensation for accurate
 *          velocity estimation.
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_MAV_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_HAL/utility/OwnPtr.h>

/**
 * @class AP_OpticalFlow_MAV
 * @brief MAVLink-based optical flow sensor backend
 * 
 * @details Receives OPTICAL_FLOW messages via MAVLink and processes flow measurements
 *          for integration into the ArduPilot navigation system. This backend:
 *          
 *          - Accumulates flow deltas (flow_x, flow_y) between update() calls
 *          - Averages quality metrics across multiple messages
 *          - Accumulates gyro samples for velocity compensation
 *          - Validates data freshness with 0.5s timeout
 *          - Converts accumulated integer flow values to angular rates (rad/s)
 *          - Applies sensor orientation transformation
 *          
 *          Multiple MAVLink messages can arrive between update() calls. The backend
 *          accumulates all flow data and computes averages when update() is called
 *          by the scheduler.
 *          
 *          Flow data is received in sensor body frame and transformed according to
 *          the configured FLOW_ORIENT_YAW parameter.
 *          
 * @note Requires HAL_GCS_ENABLED compile flag for MAVLink support
 * @warning 0.5s timeout enforced - stale data is rejected and sensor marked unhealthy
 */
class AP_OpticalFlow_MAV : public OpticalFlow_backend
{
public:
    /**
     * @brief Constructor - inherits from OpticalFlow_backend
     * 
     * @details Uses the base class OpticalFlow_backend constructor.
     *          No additional initialization required in constructor as
     *          the backend passively waits for MAVLink messages.
     */
    using OpticalFlow_backend::OpticalFlow_backend;

    /**
     * @brief Initialize the sensor (no-op for MAVLink backend)
     * 
     * @details No initialization required for MAVLink reception.
     *          The backend begins operating as soon as MAVLink
     *          OPTICAL_FLOW messages are received via handle_msg().
     *          Hardware initialization is handled by the external
     *          flow processor sending the MAVLink messages.
     */
    void init() override {}

    /**
     * @brief Process accumulated flow measurements and update frontend
     * 
     * @details Called by the scheduler to process flow data accumulated
     *          since the last update() call. This method:
     *          
     *          1. Computes dt from timestamps (prev_frame_us to latest_frame_us)
     *          2. Validates timeout: rejects data if dt > 0.5s (500ms max)
     *          3. Averages accumulated flow_x and flow_y values
     *          4. Averages quality metrics across all received messages
     *          5. Averages gyro samples for velocity compensation
     *          6. Converts accumulated integer flow deltas to angular rates (rad/s)
     *          7. Calls _update_frontend() to push data to the flow library
     *          8. Resets accumulators for next update cycle
     *          
     *          Flow rates are computed as: rate = accumulated_flow / dt
     *          
     *          Multiple MAVLink messages received between update() calls are
     *          accumulated and averaged to provide smooth, accurate measurements.
     *          
     * @note Called at scheduler rate (typically 10-50Hz depending on vehicle)
     * @warning Enforces 0.5s timeout - older data is rejected and sensor marked unhealthy
     */
    void update(void) override;

    /**
     * @brief MAVLink message callback for OPTICAL_FLOW messages
     * 
     * @details Receives and processes MAVLink OPTICAL_FLOW messages (ID 100).
     *          Decodes the message and accumulates flow data:
     *          
     *          - flow_x: Accumulated to flow_sum.x (integer pixel deltas)
     *          - flow_y: Accumulated to flow_sum.y (integer pixel deltas)
     *          - quality: Accumulated to quality_sum (0-255 scale)
     *          - sensor_id: Stored for identification
     *          - time_usec: Updates latest_frame_us timestamp
     *          
     *          Called asynchronously when MAVLink messages arrive on any telemetry link.
     *          Multiple messages can arrive between update() calls and are accumulated.
     *          
     *          Gyro data is also accumulated during message processing for velocity
     *          compensation calculations.
     *          
     * @param[in] msg MAVLink message containing OPTICAL_FLOW data (message ID 100)
     *                Format: flow_x, flow_y (integer), quality (0-255), sensor_id, time_usec
     *                
     * @note Thread-safe accumulation of flow data for processing in update()
     * @note Gyro compensation data accumulated separately for averaging
     */
    void handle_msg(const mavlink_message_t &msg) override;

    /**
     * @brief Static factory method to create MAVLink flow backend
     * 
     * @details Creates a new instance of the MAVLink optical flow backend.
     *          This method always succeeds as the backend passively waits for
     *          MAVLink messages - no hardware detection is required.
     *          
     *          The backend becomes active when the first OPTICAL_FLOW message
     *          is received via handle_msg().
     *          
     * @param[in] _frontend Reference to the AP_OpticalFlow manager instance
     *                      
     * @return Pointer to new AP_OpticalFlow_MAV instance (always succeeds)
     *         
     * @note Unlike hardware backends, this always returns a valid instance
     *       since no physical sensor detection is needed
     */
    static AP_OpticalFlow_MAV *detect(AP_OpticalFlow &_frontend);

private:

    /**
     * Previous update timestamp for dt calculation
     * Units: microseconds (uint64_t)
     * Used to compute time delta between update() calls for flow rate conversion
     */
    uint64_t prev_frame_us;
    
    /**
     * Most recent MAVLink message timestamp
     * Units: microseconds (uint64_t)
     * Updated by handle_msg() when OPTICAL_FLOW messages arrive
     * Used for timeout validation (0.5s max) and dt calculation
     */
    uint64_t latest_frame_us;
    
    /**
     * Accumulated flow deltas since last update()
     * Units: Integer pixel deltas (flow_x, flow_y from MAVLink message)
     * Summed across all messages received between update() calls
     * Converted to angular rates (rad/s) in update() by dividing by dt
     * Coordinate frame: Sensor body frame (transformed by FLOW_ORIENT_YAW)
     */
    Vector2l flow_sum;
    
    /**
     * Accumulated quality values for averaging
     * Units: Sum of quality values (0-255 scale from MAVLink messages)
     * Divided by count in update() to compute average quality
     * Quality 0 = no confidence, 255 = maximum confidence
     */
    uint16_t quality_sum;
    
    /**
     * Number of messages received since last update()
     * Used to average flow_sum and quality_sum values
     * Reset to 0 after each update() call
     */
    uint16_t count;
    
    /**
     * Sensor identifier from MAVLink message
     * Allows distinguishing between multiple optical flow sensors
     * Stored from most recent OPTICAL_FLOW message
     */
    uint8_t sensor_id;
    
    /**
     * Accumulated gyro values for velocity compensation
     * Units: rad/s accumulated across gyro samples
     * Averaged and used to compensate flow measurements for vehicle rotation
     * Coordinate frame: Body frame (same as flow measurements)
     */
    Vector2f gyro_sum;
    
    /**
     * Number of gyro samples in gyro_sum accumulator
     * Used to compute average gyro rate for velocity compensation
     * Reset after each update() call
     */
    uint16_t gyro_sum_count;
};

#endif  // AP_OPTICALFLOW_MAV_ENABLED
