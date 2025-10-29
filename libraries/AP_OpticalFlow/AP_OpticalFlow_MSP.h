/**
 * @file AP_OpticalFlow_MSP.h
 * @brief MSP (MultiWii Serial Protocol) optical flow sensor backend
 * 
 * @details This backend receives optical flow data via the MSP protocol,
 *          typically from external processors or MSP-based OSD systems that
 *          provide optical flow measurements. The MSP protocol is shared with
 *          on-screen display systems and other peripherals.
 * 
 *          Protocol: MSP OPTICAL_FLOW packet format
 *          Communication: MSP serial protocol (typically shared with OSD)
 *          Use case: Integration with MSP-based OSD systems and external processors
 * 
 * @note Requires HAL_MSP_OPTICALFLOW_ENABLED compile flag
 * @warning Axis convention differs from other backends - inverts both axes
 * 
 * Source: libraries/AP_OpticalFlow/AP_OpticalFlow_MSP.h
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if HAL_MSP_OPTICALFLOW_ENABLED

#include "AP_OpticalFlow.h"

/**
 * @class AP_OpticalFlow_MSP
 * @brief MSP protocol optical flow sensor backend
 * 
 * @details Receives MSP optical flow packets via handle_msp(), accumulates
 *          integer flow deltas and quality between update() calls, averages
 *          gyro samples for velocity compensation, applies yaw rotation and
 *          axis inversions as needed.
 * 
 *          Algorithm:
 *          1. handle_msp() accumulates flow_x/flow_y/quality from MSP packets
 *          2. update() processes accumulated data:
 *             - Computes time delta (dt) from prev_frame_us to latest_frame_us
 *             - Averages gyro samples for compensation
 *             - Scales and inverts axes (MSP convention)
 *             - Applies yaw rotation via _applyYaw()
 *             - Updates frontend via _update_frontend()
 * 
 *          Coordinate frames: MSP provides body frame flow, backend applies
 *          yaw and axis inversions per ArduPilot conventions
 * 
 * @note Multiple MSP packets are accumulated between update() calls
 * @warning Axis convention differs from other backends - inverts both axes
 *          (specific to MSP OSD convention)
 */
class AP_OpticalFlow_MSP : public OpticalFlow_backend
{
public:
    /**
     * @brief Constructor - uses base class OpticalFlow_backend constructor
     * 
     * @details Inherits constructor from OpticalFlow_backend to initialize
     *          the backend with reference to the frontend manager
     */
    using OpticalFlow_backend::OpticalFlow_backend;

    /**
     * @brief Initialize the MSP optical flow sensor backend
     * 
     * @details No initialization required for MSP reception. The backend is
     *          ready to receive MSP packets immediately after construction.
     *          All state is reset on first handle_msp() call.
     * 
     * @note Override of OpticalFlow_backend::init()
     */
    void init() override {}

    /**
     * @brief Process accumulated optical flow data and update frontend
     * 
     * @details Processes all MSP optical flow packets received since the last
     *          update() call. Computes time delta (dt) between frames, averages
     *          accumulated flow deltas and quality values, averages gyro samples
     *          for velocity compensation, applies axis scaling and inversions,
     *          and applies yaw rotation before updating the frontend.
     * 
     *          Algorithm:
     *          1. Check if new data available (count > 0)
     *          2. Calculate dt from prev_frame_us to latest_frame_us (microseconds)
     *          3. Average flow_sum, quality_sum, and gyro_sum by count
     *          4. Scale flow values using frontend _flowScaler()
     *          5. Invert both axes (MSP convention: axes inverted relative to other sensors)
     *          6. Apply yaw rotation via _applyYaw() using _yawAngleRad()
     *          7. Update frontend via _update_frontend() with flow rates (rad/s) and quality
     *          8. Reset accumulators for next cycle
     * 
     * @note Called at main loop rate by frontend, typically processes multiple MSP packets per call
     * @note Units: Converts integer pixel deltas to rad/s flow rates
     * @warning Axis convention differs from other backends - inverts both axes
     */
    void update(void) override;

    /**
     * @brief Handle incoming MSP optical flow packet
     * 
     * @details MSP packet callback that accumulates flow data from external
     *          processors or MSP-based systems. Receives integer flow deltas
     *          (flow_x, flow_y) and quality value from MSP OPTICAL_FLOW message.
     *          Accumulates values in flow_sum and quality_sum for averaging in
     *          update(). Updates latest_frame_us timestamp.
     * 
     * @param[in] pkt MSP optical flow data message containing:
     *                - flow_x: Integer horizontal flow delta (pixels)
     *                - flow_y: Integer vertical flow delta (pixels)
     *                - quality: Flow quality metric (0-255 scale)
     * 
     * @note Called by MSP protocol handler when OPTICAL_FLOW message received
     * @note Multiple packets may be received between update() calls
     * @note Units: flow_x/flow_y are integer pixel deltas, quality is 0-255
     */
    void handle_msp(const MSP::msp_opflow_data_message_t &pkt) override;

    /**
     * @brief Static factory method to detect and create MSP optical flow backend
     * 
     * @details Creates a new AP_OpticalFlow_MSP instance. Always succeeds since
     *          MSP optical flow detection is based on receiving MSP packets rather
     *          than hardware probing. The backend becomes active when MSP OPTICAL_FLOW
     *          messages are received via handle_msp().
     * 
     * @param[in] _frontend Reference to AP_OpticalFlow frontend manager
     * 
     * @return Pointer to new AP_OpticalFlow_MSP instance (always succeeds)
     * 
     * @note Does not perform hardware detection - sensor presence determined by packet reception
     */
    static AP_OpticalFlow_MSP *detect(AP_OpticalFlow &_frontend);

private:

    /**
     * Previous frame timestamp for dt calculation
     * Units: microseconds (uint64_t)
     * Purpose: Timestamp of last update() call, used to compute time delta (dt)
     *          between optical flow measurements for rate calculation
     */
    uint64_t prev_frame_us;

    /**
     * Most recent MSP packet timestamp
     * Units: microseconds (uint64_t)
     * Purpose: Timestamp of most recent handle_msp() call, becomes prev_frame_us
     *          after update() processes accumulated data
     */
    uint64_t latest_frame_us;

    /**
     * Accumulated integer flow deltas since last update()
     * Units: Integer pixel deltas (Vector2l)
     * Purpose: Sum of flow_x and flow_y from all MSP packets received since
     *          last update() call. Averaged by count and converted to rad/s in update().
     * Coordinate frame: Body frame (sensor coordinates), inverted in update()
     */
    Vector2l flow_sum;

    /**
     * Accumulated quality values since last update()
     * Units: Dimensionless (0-255 scale per packet, summed)
     * Purpose: Sum of quality values from all MSP packets since last update().
     *          Averaged by count in update() to produce mean quality metric.
     */
    uint16_t quality_sum;

    /**
     * Number of MSP packets received since last update()
     * Units: Count (dimensionless)
     * Purpose: Number of handle_msp() calls since last update(), used to
     *          average flow_sum, quality_sum, and gyro_sum
     */
    uint16_t count;

    /**
     * Accumulated gyro samples for velocity compensation
     * Units: rad/s (Vector2f, summed)
     * Purpose: Sum of gyro sensor values since last optical flow frame, used
     *          for gyro-based velocity compensation. Averaged by gyro_sum_count
     *          in update() before applying to flow calculation.
     * Coordinate frame: Body frame (roll/pitch rates)
     */
    Vector2f gyro_sum;

    /**
     * Number of gyro samples in gyro_sum accumulator
     * Units: Count (dimensionless)
     * Purpose: Number of gyro samples accumulated in gyro_sum, used to compute
     *          average gyro value for velocity compensation in update()
     */
    uint16_t gyro_sum_count;
};

#endif // HAL_MSP_OPTICALFLOW_ENABLED
