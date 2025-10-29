/**
 * @file AP_RangeFinder_Benewake_TF03.h
 * @brief Benewake TF03 180-meter range serial LiDAR rangefinder backend driver
 * 
 * This file implements the driver for the Benewake TF03 LiDAR distance sensor,
 * which provides distance measurements up to 180 meters via serial communication.
 * The TF03 uses the standard Benewake serial protocol implemented in the parent
 * AP_RangeFinder_Benewake class.
 * 
 * Hardware specifications:
 * - Maximum range: 180 meters (18000 cm)
 * - Interface: Serial UART
 * - Protocol: Benewake standard serial protocol
 * - Sensor type: Laser rangefinder (LiDAR)
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TF03.h
 * 
 * @note This driver is conditionally compiled based on AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
 * @see AP_RangeFinder_Benewake for protocol implementation details
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED

#include "AP_RangeFinder_Benewake.h"

/**
 * @class AP_RangeFinder_Benewake_TF03
 * @brief Rangefinder backend driver for Benewake TF03 180m LiDAR sensor
 * 
 * @details This class implements support for the Benewake TF03 long-range LiDAR
 *          distance sensor. The TF03 is designed for long-range distance measurement
 *          applications up to 180 meters, making it suitable for terrain following
 *          in fixed-wing aircraft and high-altitude altitude hold.
 * 
 *          The driver inherits the serial protocol handling from AP_RangeFinder_Benewake
 *          and only needs to specify the maximum detection range specific to the TF03 model.
 * 
 *          Key characteristics:
 *          - Maximum detection range: 180 meters (18000 centimeters)
 *          - Serial communication via UART interface
 *          - Laser-based distance measurement (LiDAR)
 *          - Suitable for long-range terrain following and altitude measurement
 * 
 *          Usage in vehicle:
 *          - Set RNGFNDx_TYPE = 20 (Benewake TFmini/Plus/TF02/TF03)
 *          - Configure serial port for rangefinder communication
 *          - Sensor automatically detected as TF03 based on configuration
 * 
 * @note This class uses the standard Benewake protocol implementation from the parent class
 * @note Factory pattern used for instantiation via create() method
 * 
 * @see AP_RangeFinder_Benewake for base protocol implementation
 * @see AP_RangeFinder_Backend_Serial for serial communication base
 */
class AP_RangeFinder_Benewake_TF03 : public AP_RangeFinder_Benewake
{
public:

    /**
     * @brief Factory method to create an instance of the TF03 rangefinder backend
     * 
     * @details This static factory method creates and returns a new instance of the
     *          AP_RangeFinder_Benewake_TF03 driver. It uses NEW_NOTHROW to safely
     *          allocate memory, returning nullptr if allocation fails.
     * 
     *          This method is called by the rangefinder library when a TF03 sensor
     *          is configured via parameters (RNGFNDx_TYPE = 20 for Benewake sensors).
     * 
     * @param[in,out] _state     Reference to rangefinder state structure containing
     *                           current distance, status, and health information
     * @param[in]     _params    Reference to rangefinder configuration parameters
     *                           including serial port, orientation, scaling, etc.
     * 
     * @return Pointer to newly created AP_RangeFinder_Benewake_TF03 backend instance,
     *         or nullptr if memory allocation fails
     * 
     * @note Memory allocation uses NEW_NOTHROW to prevent exceptions on allocation failure
     * @note Caller is responsible for checking return value for nullptr
     * 
     * @see AP_RangeFinder_Backend_Serial base class
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Benewake_TF03(_state, _params);
    }

protected:
    /**
     * @brief Get the maximum distance measurement capability of the TF03 sensor
     * 
     * @details Returns the maximum detection range of the Benewake TF03 LiDAR sensor
     *          in centimeters. The TF03 model has a maximum range of 180 meters,
     *          which is significantly longer than other Benewake models like the
     *          TFmini (12m) or TF02 (22m).
     * 
     *          This value is used by the rangefinder library to:
     *          - Validate measurements (reject readings beyond maximum range)
     *          - Set the RNGFNDx_MAX_CM parameter if not configured
     *          - Determine sensor capability for mission planning
     * 
     * @return Maximum detection distance in centimeters (18000 cm = 180 meters)
     * 
     * @note This method overrides the pure virtual function from AP_RangeFinder_Benewake
     * @note Value is constant and determined by TF03 hardware specifications
     * @note Actual usable range may be less depending on target reflectivity and environmental conditions
     */
    float model_dist_max_cm() const override { return 18000; }

private:
    /**
     * @brief Inherit constructor from parent AP_RangeFinder_Benewake class
     * 
     * @details Uses C++11 constructor inheritance to make the parent class constructor
     *          available for this derived class. This allows the TF03 driver to use
     *          the standard Benewake constructor without reimplementing it.
     * 
     *          The inherited constructor accepts:
     *          - RangeFinder::RangeFinder_State reference (sensor state)
     *          - AP_RangeFinder_Params reference (configuration parameters)
     * 
     * @note Private inheritance used by the factory method create()
     * @see AP_RangeFinder_Benewake::AP_RangeFinder_Benewake() base constructor
     */
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
};

#endif  // AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
