/**
 * @file AP_RangeFinder_Benewake_TFMini.h
 * @brief Benewake TFmini serial LiDAR rangefinder backend driver
 * 
 * This driver implements support for the Benewake TFmini, a compact and low-cost
 * 12-meter range serial LiDAR sensor. The TFmini communicates over serial UART
 * at 115200 baud using the Benewake binary protocol.
 * 
 * Hardware Specifications:
 * - Maximum Range: 12 meters
 * - Serial Interface: UART, 115200 baud (8N1)
 * - Protocol: Benewake binary format
 * - Sensor Type: LiDAR (Time-of-Flight)
 * - Form Factor: Compact, lightweight
 * - Cost: Low-cost option for short-range distance sensing
 * 
 * The TFmini is commonly used for:
 * - Altitude hold (low altitude applications)
 * - Obstacle detection
 * - Terrain following (short range)
 * - Indoor positioning
 * - Landing assistance
 * 
 * @note This sensor is limited to 12 meters maximum range, suitable for
 *       low-altitude and close-range applications only
 * 
 * @see AP_RangeFinder_Benewake for base Benewake protocol implementation
 * @see AP_RangeFinder_Backend_Serial for serial communication infrastructure
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED

#include "AP_RangeFinder_Benewake.h"

/**
 * @class AP_RangeFinder_Benewake_TFMini
 * @brief Driver backend for Benewake TFmini 12-meter serial LiDAR sensor
 * 
 * @details This class implements the rangefinder backend for the Benewake TFmini,
 *          a compact 12-meter LiDAR sensor. It inherits from AP_RangeFinder_Benewake
 *          which provides the Benewake binary protocol parsing. This implementation
 *          only needs to specify the model-specific maximum range.
 * 
 *          The TFmini operates at 115200 baud and uses the standard Benewake
 *          binary protocol. All protocol handling, serial communication, and
 *          range reading are implemented in the base class.
 * 
 * Hardware Configuration:
 * - Connect to any available UART port
 * - Set SERIALx_PROTOCOL = 9 (Rangefinder)
 * - Set RNGFND1_TYPE = 20 (Benewake TFmini)
 * - Sensor automatically operates at 115200 baud
 * 
 * @note This is a factory pattern implementation - use the create() static method
 *       to instantiate the driver
 * 
 * @warning 12-meter maximum range limitation - not suitable for high-altitude
 *          altitude hold or long-range terrain following
 */
class AP_RangeFinder_Benewake_TFMini : public AP_RangeFinder_Benewake
{
public:

    /**
     * @brief Factory method to create a TFmini rangefinder backend instance
     * 
     * @details This static factory method instantiates a new AP_RangeFinder_Benewake_TFMini
     *          driver backend using dynamic allocation. The factory pattern allows the
     *          rangefinder library to create the appropriate backend type without knowing
     *          the specific implementation details.
     * 
     *          Memory is allocated using NEW_NOTHROW to safely handle allocation failures
     *          in memory-constrained embedded systems. If allocation fails, returns nullptr.
     * 
     * @param[in,out] _state   Reference to rangefinder state structure containing current
     *                         readings, status, and configuration. Updated by driver as
     *                         new measurements are received.
     * @param[in,out] _params  Reference to rangefinder parameters including orientation,
     *                         address, pin configuration, and calibration values.
     * 
     * @return Pointer to newly created TFmini backend instance, or nullptr if allocation fails
     * 
     * @note This method is called by the rangefinder library during sensor initialization
     * @note Memory is managed by the rangefinder library - do not manually delete
     * 
     * @see AP_RangeFinder::detect_instance() for driver instantiation logic
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Benewake_TFMini(_state, _params);
    }

protected:
    /**
     * @brief Returns the maximum distance measurement capability for TFmini sensor
     * 
     * @details This method specifies the TFmini's maximum range of 12 meters (1200 cm).
     *          The base class uses this value to:
     *          - Validate incoming range measurements
     *          - Report sensor capabilities to the system
     *          - Scale distance values appropriately
     *          - Set MAVLink DISTANCE_SENSOR.max_distance field
     * 
     *          The TFmini hardware specification limits maximum range to 12 meters,
     *          though practical range may be less depending on target surface properties
     *          (reflectivity, angle, ambient light conditions).
     * 
     * @return Maximum distance in centimeters (1200 cm = 12 meters)
     * 
     * @note This is a compile-time constant - value is fixed for TFmini hardware
     * @note Readings beyond this range are considered invalid by the base class
     * 
     * @warning Do not modify this value - it reflects the actual hardware limitation
     *          of the TFmini sensor. Increasing this value will not extend sensor range.
     */
    float model_dist_max_cm() const override { return 1200; }

private:

    /**
     * @brief Inherit constructor from base Benewake class
     * 
     * @details Uses the AP_RangeFinder_Benewake constructor directly since no
     *          TFmini-specific initialization is required. All serial communication
     *          and protocol handling is implemented in the base class.
     * 
     *          The base class constructor initializes:
     *          - Serial port configuration (115200 baud, 8N1)
     *          - Protocol parser state
     *          - Line buffer for incoming data
     *          - Range measurement state
     */
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
};

#endif  // AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
