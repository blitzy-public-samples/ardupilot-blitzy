/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_ENABLED

/**
 * @file AP_OpticalFlow.h
 * @brief Optical flow sensor frontend manager and main API for ArduPilot
 * 
 * @details This file defines the AP_OpticalFlow class, which serves as the top-level
 *          interface for the optical flow sensor subsystem. It manages the lifecycle
 *          of optical flow backend drivers, provides a unified API for vehicle code
 *          and the Extended Kalman Filter (EKF), and handles parameter storage and
 *          validation.
 * 
 *          Key Responsibilities:
 *          - Backend lifecycle management (initialization, updates, destruction)
 *          - Parameter management (sensor type, scaling, orientation, position offset)
 *          - State aggregation from backend measurements
 *          - EKF integration via writeOptFlowMeas() to AHRS
 *          - MAVLink and MSP message routing to appropriate backend
 *          - Calibration interface for automatic sensor correction
 *          - Binary logging when HAL_LOGGING_ENABLED
 * 
 *          Singleton Pattern:
 *          Access via AP::opticalflow() - only one instance exists per system.
 * 
 *          Supported Backend Types:
 *          - PX4Flow: I2C optical flow sensor with integrated sonar
 *          - Pixart: PMW3900/3901 SPI sensors for precision flow measurement
 *          - CXOF: Cheerson CX-OF UART protocol
 *          - MAVLink: Flow data from external MAVLink sources
 *          - DroneCAN: HereFlow and other CAN-based flow sensors
 *          - MSP: Flow data from MSP protocol devices
 *          - UPFLOW: UPixel UPFLOW UART sensors
 *          - SITL: Simulation backend for Software-in-the-Loop testing
 * 
 * @note This is a singleton class accessed via AP::opticalflow()
 * @warning Optical flow requires proper surface texture (high contrast features)
 *          and appropriate lighting conditions for reliable operation
 * @warning Accurate velocity estimation requires rangefinder or FLOW_HGT_OVR parameter
 */

#include <AP_MSP/msp.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_OpticalFlow_Calibrator.h"

class OpticalFlow_backend;

/**
 * @class AP_OpticalFlow
 * @brief Optical flow sensor frontend manager and main API
 * 
 * @details AP_OpticalFlow manages the optical flow sensor subsystem, providing a unified
 *          interface for vehicle code and navigation filters to access flow measurements.
 *          The class follows a frontend-backend architecture pattern where this frontend
 *          class handles parameter management, state aggregation, and external interfaces,
 *          while backend classes implement sensor-specific communication protocols.
 * 
 *          Architecture:
 *          - Frontend (this class): Parameter storage, API interface, state management
 *          - Backend (OpticalFlow_backend subclasses): Sensor-specific drivers
 * 
 *          Parameter System:
 *          - FLOW_TYPE: Selects backend driver (0=NONE, 1=PX4Flow, 2=Pixart, etc.)
 *          - FLOW_FXSCALER/FYSCALER: X/Y axis scaling correction (parts per thousand)
 *          - FLOW_ORIENT_YAW: Sensor yaw orientation in centidegrees
 *          - FLOW_POS_X/Y/Z: Sensor position offset from IMU (meters, body frame)
 *          - FLOW_ADDR: I2C address for applicable sensors (0-127)
 *          - FLOW_HGT_OVR: Height override for rovers (meters, 0=use rangefinder)
 * 
 *          Flow-to-Velocity Conversion:
 *          The fundamental equation for optical flow is:
 *              velocity = flowRate * height_above_ground
 *          The EKF performs this conversion using either rangefinder data or FLOW_HGT_OVR,
 *          then fuses the velocity estimate with GPS, IMU, and other sensor data.
 * 
 *          Coordinate Frames:
 *          - Body frame: X forward, Y right, Z down (NED convention)
 *          - flowRate: Angular rates about X/Y body axes (rad/s, right-hand rule)
 *          - Sensor orientation applied via FLOW_ORIENT_YAW rotation
 * 
 *          Integration Points:
 *          - AHRS/EKF: Measurements forwarded via writeOptFlowMeas() for velocity estimation
 *          - Vehicle code: Status queries via quality(), healthy(), flowRate()
 *          - GCS: MAVLink messages routed via handle_msg()
 *          - Logging: Binary log messages via Log_Write_Optflow() when enabled
 * 
 *          Typical Usage Workflow:
 *          1. Set FLOW_TYPE parameter to select sensor (e.g., 1=PX4Flow, 2=Pixart)
 *          2. Configure FLOW_POS_X/Y/Z for precise EKF fusion (sensor offset from IMU)
 *          3. Adjust FLOW_ORIENT_YAW if sensor mounted at non-zero yaw angle
 *          4. Calibrate with FLOW_FXSCALER/FYSCALER if needed (typically -200 to +200)
 *          5. For rovers: Set FLOW_HGT_OVR to fixed sensor height if no rangefinder
 *          6. Vehicle scheduler calls update() periodically (10-50Hz)
 *          7. EKF automatically receives measurements for velocity estimation
 * 
 * @note Singleton pattern - access via AP::opticalflow(), not direct instantiation
 * @note Backend selection occurs at init() based on FLOW_TYPE parameter
 * @note Thread-safety: Update method assumes single-threaded scheduler context
 * 
 * @warning Surface texture requirements: Optical flow requires high-contrast features
 *          on the surface below. Uniform surfaces (blank walls, calm water) will produce
 *          low quality measurements and should trigger quality() to return low values.
 * 
 * @warning Lighting requirements: Most sensors require adequate lighting but not direct
 *          sunlight. Infrared-based sensors may work in low light conditions.
 * 
 * @warning Height dependency: Flow accuracy degrades at high altitudes. Most sensors
 *          work reliably up to 5-10 meters depending on surface texture and lighting.
 * 
 * @warning EKF integration: Requires AP_AHRS_ENABLED and proper EKF configuration.
 *          Incorrect FLOW_POS_X/Y/Z values can cause navigation errors.
 */
class AP_OpticalFlow
{
    /**
     * @note OpticalFlow_backend is declared as friend to allow backend classes to:
     *       - Call update_state() to push measurements to frontend
     *       - Access protected parameters for sensor configuration
     *       - Access internal state for logging and calibration
     */
    friend class OpticalFlow_backend;

public:
    /**
     * @brief Constructor - Initialize optical flow frontend singleton
     * 
     * @details Initializes the singleton instance, registers parameters with AP_Param
     *          system, and sets initial state. The constructor does not initialize
     *          the backend - that occurs in init() based on the FLOW_TYPE parameter.
     * 
     * @note This constructor is called once during system startup to create the singleton
     * @note Backend initialization is deferred to init() to allow parameter loading first
     */
    AP_OpticalFlow();

    /**
     * @brief Prevent copy construction - singleton pattern enforcement
     * 
     * @note Optical flow is a singleton; copying would violate this design pattern
     */
    CLASS_NO_COPY(AP_OpticalFlow);

    /**
     * @brief Static singleton accessor
     * 
     * @details Returns pointer to the single AP_OpticalFlow instance. Prefer using
     *          the namespace accessor AP::opticalflow() in application code for
     *          cleaner syntax.
     * 
     * @return AP_OpticalFlow* Pointer to singleton instance, or nullptr if not created
     * 
     * @note Prefer AP::opticalflow() for cleaner code
     * @see AP::opticalflow()
     */
    static AP_OpticalFlow *get_singleton() {
        return _singleton;
    }

    /**
     * @enum Type
     * @brief Optical flow sensor type enumeration (FLOW_TYPE parameter)
     * 
     * @details Defines the available optical flow sensor backends. The FLOW_TYPE parameter
     *          selects which backend to instantiate during init(). Each backend implements
     *          sensor-specific communication protocols and data processing.
     * 
     *          Sensor Selection Criteria:
     *          - PX4Flow: I2C sensor with integrated sonar, good general purpose option
     *          - Pixart: High-speed SPI sensors (PMW3900/3901) for racing/precision
     *          - BEBOP: On-board HAL implementation for Parrot Bebop platform
     *          - CXOF: UART-based Cheerson CX-OF sensor
     *          - MAVLINK: External MAVLink source (companion computer, offboard processing)
     *          - UAVCAN/DroneCAN: CAN bus sensors like HereFlow (integrated GPS+flow)
     *          - MSP: MSP protocol integration for OSD and flight controller communication
     *          - UPFLOW: UART-based UPixel UPFLOW sensor
     *          - SITL: Simulation backend for testing without hardware
     * 
     * @note Not all backends are available in all builds - conditional compilation via feature flags
     * @note Default value is NONE (optical flow disabled)
     */
    enum class Type {
        /**
         * @brief No optical flow sensor / disabled
         * @details When FLOW_TYPE=0, optical flow subsystem is completely disabled.
         *          No backend is instantiated, and enabled() returns false.
         */
        NONE = 0,
#if AP_OPTICALFLOW_PX4FLOW_ENABLED
        /**
         * @brief PX4Flow I2C optical flow sensor
         * @details PX4Flow is an I2C sensor combining optical flow camera with ultrasonic
         *          rangefinder. Provides flow rate, gyro data, and distance measurements.
         *          Default I2C address is 0x42 (configurable via FLOW_ADDR parameter).
         * @note Requires AP_OPTICALFLOW_PX4FLOW_ENABLED feature flag
         */
        PX4FLOW = 1,
#endif  // AP_OPTICALFLOW_PX4FLOW_ENABLED
#if AP_OPTICALFLOW_PIXART_ENABLED
        /**
         * @brief Pixart PMW3900/3901 SPI optical flow sensor
         * @details High-speed SPI optical flow sensor from Pixart. PMW3900/3901 chips
         *          offer high update rates suitable for racing and precision applications.
         *          Typically mounted facing downward for velocity estimation.
         * @note Requires AP_OPTICALFLOW_PIXART_ENABLED feature flag
         */
        PIXART = 2,
#endif  // AP_OPTICALFLOW_PIXART_ENABLED
#if AP_OPTICALFLOW_ONBOARD_ENABLED
        /**
         * @brief On-board HAL optical flow (Parrot Bebop)
         * @details Uses platform-specific HAL implementation for optical flow processing.
         *          Originally developed for Parrot Bebop drone's bottom-facing camera.
         * @note Requires AP_OPTICALFLOW_ONBOARD_ENABLED feature flag
         * @note Platform-specific - only available on supported boards
         */
        BEBOP = 3,
#endif  // AP_OPTICALFLOW_ONBOARD_ENABLED
#if AP_OPTICALFLOW_CXOF_ENABLED
        /**
         * @brief Cheerson CX-OF UART optical flow sensor
         * @details UART-based optical flow sensor using Cheerson CX-OF protocol.
         *          Communicates via serial port with custom binary protocol.
         * @note Requires AP_OPTICALFLOW_CXOF_ENABLED feature flag
         */
        CXOF = 4,
#endif  // AP_OPTICALFLOW_CXOF_ENABLED
#if AP_OPTICALFLOW_MAV_ENABLED
        /**
         * @brief MAVLink OPTICAL_FLOW message source
         * @details Receives optical flow data via MAVLink OPTICAL_FLOW messages.
         *          Useful for companion computer-based flow processing or external
         *          sensor systems with MAVLink output.
         * @note Requires AP_OPTICALFLOW_MAV_ENABLED feature flag
         * @see handle_msg() for MAVLink message routing
         */
        MAVLINK = 5,
#endif  // AP_OPTICALFLOW_MAV_ENABLED
#if AP_OPTICALFLOW_HEREFLOW_ENABLED
        /**
         * @brief DroneCAN/UAVCAN optical flow sensor (HereFlow)
         * @details CAN bus optical flow sensor using DroneCAN protocol. HereFlow is
         *          a common implementation combining optical flow with GPS and magnetometer.
         *          Supports firmware updates and configuration over CAN.
         * @note Requires AP_OPTICALFLOW_HEREFLOW_ENABLED feature flag
         */
        UAVCAN = 6,
#endif  // AP_OPTICALFLOW_HEREFLOW_ENABLED
#if HAL_MSP_OPTICALFLOW_ENABLED
        /**
         * @brief MSP (MultiWii Serial Protocol) optical flow
         * @details Receives optical flow data via MSP protocol messages. Used for
         *          integration with MSP-based OSD systems and flight controllers.
         * @note Requires HAL_MSP_OPTICALFLOW_ENABLED feature flag
         * @see handle_msp() for MSP message routing
         */
        MSP = 7,
#endif  // HAL_MSP_OPTICALFLOW_ENABLED
#if AP_OPTICALFLOW_UPFLOW_ENABLED
        /**
         * @brief UPixel UPFLOW UART optical flow sensor
         * @details UART-based UPixel UPFLOW optical flow sensor. Communicates via
         *          serial port with UPixel-specific protocol.
         * @note Requires AP_OPTICALFLOW_UPFLOW_ENABLED feature flag
         */
        UPFLOW = 8,
#endif  // AP_OPTICALFLOW_UPFLOW_ENABLED
#if AP_OPTICALFLOW_SITL_ENABLED
        /**
         * @brief Software-in-the-Loop (SITL) simulation backend
         * @details Simulation backend for testing optical flow integration without
         *          physical hardware. Generates synthetic flow measurements based on
         *          simulated vehicle motion and terrain.
         * @note Requires AP_OPTICALFLOW_SITL_ENABLED feature flag
         * @note Only available in SITL builds, not on actual hardware
         */
        SITL = 10,
#endif  // AP_OPTICALFLOW_SITL_ENABLED
    };

    /**
     * @brief Initialize optical flow backend based on FLOW_TYPE parameter
     * 
     * @details Called once during vehicle startup after parameters are loaded. Instantiates
     *          the appropriate backend driver based on the FLOW_TYPE parameter value.
     *          The backend's detect() factory method is called first to verify sensor
     *          presence, then the backend is allocated with NEW_NOTHROW.
     * 
     *          Initialization Sequence:
     *          1. Check if FLOW_TYPE != NONE
     *          2. Call backend-specific detect() factory method
     *          3. If detection succeeds, allocate backend with NEW_NOTHROW
     *          4. Backend performs sensor-specific initialization
     *          5. Backend registers periodic update callback with scheduler
     * 
     * @param[in] log_bit Bitmask for selective logging. Use -1 to always log, or
     *                    specific LOG_* bit to enable conditional logging based on
     *                    LOG_BITMASK parameter
     * 
     * @note Call only once during vehicle startup after parameter loading
     * @note If backend allocation fails (out of memory), sensor remains disabled
     * @warning Must be called after AP_Param system initialization to read FLOW_TYPE
     */
    void init(uint32_t log_bit);

    /**
     * @brief Check if optical flow is enabled
     * 
     * @details Returns true if FLOW_TYPE parameter is set to a value other than NONE.
     *          This indicates that the user has configured an optical flow sensor,
     *          though the sensor may not necessarily be healthy or connected.
     * 
     * @return true if FLOW_TYPE != NONE (sensor type configured)
     * @return false if FLOW_TYPE == NONE (optical flow disabled)
     * 
     * @note Enabled does not imply healthy - use healthy() to check sensor status
     * @see healthy()
     */
    bool enabled() const { return _type != Type::NONE; }

    /**
     * @brief Check if optical flow sensor is healthy
     * 
     * @details Returns true if both:
     *          1. Backend has been successfully instantiated (not nullptr)
     *          2. Backend has set the healthy flag (receiving valid measurements)
     * 
     *          A healthy sensor is actively providing measurements with acceptable
     *          quality. The healthy flag is cleared if measurements stop arriving
     *          or if the backend detects communication errors.
     * 
     * @return true if backend exists and is reporting healthy status
     * @return false if no backend or backend is unhealthy
     * 
     * @note Check healthy() before using flow measurements in critical logic
     * @note Unhealthy state may be transient (temporary communication glitch)
     * @see quality() for surface quality assessment
     */
    bool healthy() const { return backend != nullptr && _flags.healthy; }

    /**
     * @brief Periodic update - call from vehicle scheduler
     * 
     * @details Called periodically (typically 10-50Hz) from the vehicle's main scheduler.
     *          Performs the following operations:
     *          1. Calls backend->update() to fetch latest sensor measurements
     *          2. Applies sensor orientation correction (FLOW_ORIENT_YAW rotation)
     *          3. Applies scaling corrections (FLOW_FXSCALER, FLOW_FYSCALER)
     *          4. Forwards measurements to AHRS via writeOptFlowMeas() when AP_AHRS_ENABLED
     *          5. Writes binary log message via Log_Write_Optflow() when logging enabled
     * 
     *          The backend's update() typically fetches data from the sensor, performs
     *          sensor-specific processing, and calls update_state() to push measurements
     *          to the frontend.
     * 
     * @note Call at consistent rate from vehicle scheduler (typically 10-50Hz)
     * @note EKF integration happens automatically when AP_AHRS_ENABLED
     * @warning Do not call from multiple threads - assumes single-threaded scheduler
     * @see update_state() for backend interface to push measurements
     */
    void update(void);

    /**
     * @brief Handle MAVLink optical flow messages
     * 
     * @details Routes MAVLink messages to the appropriate backend for processing.
     *          Used when FLOW_TYPE=MAVLINK to receive flow data via MAVLink
     *          OPTICAL_FLOW messages, typically from a companion computer or
     *          external flow processing system.
     * 
     * @param[in] msg MAVLink message structure containing OPTICAL_FLOW data
     * 
     * @note Only functional when FLOW_TYPE=MAVLINK
     * @note Backend is responsible for parsing message and extracting flow data
     * @see Type::MAVLINK
     */
    void handle_msg(const mavlink_message_t &msg);

#if HAL_MSP_OPTICALFLOW_ENABLED
    /**
     * @brief Handle MSP optical flow messages
     * 
     * @details Routes MSP (MultiWii Serial Protocol) messages to the appropriate
     *          backend for processing. Used when FLOW_TYPE=MSP to receive flow
     *          data from MSP-based systems.
     * 
     * @param[in] pkt MSP optical flow data message packet containing flow measurements
     * 
     * @note Only available when HAL_MSP_OPTICALFLOW_ENABLED
     * @note Only functional when FLOW_TYPE=MSP
     * @see Type::MSP
     */
    void handle_msp(const MSP::msp_opflow_data_message_t &pkt);
#endif

    /**
     * @brief Get surface quality metric from sensor
     * 
     * @details Returns the image/surface quality as reported by the optical flow sensor.
     *          Quality represents the confidence in the flow measurement based on the
     *          texture and contrast of the surface below the sensor.
     * 
     *          Quality Interpretation:
     *          - 0: Invalid measurement, no usable surface features detected
     *          - 1-50: Poor quality, measurements unreliable
     *          - 51-150: Acceptable quality, measurements usable with caution
     *          - 151-255: Excellent quality, high confidence in measurements
     * 
     *          Low Quality Causes:
     *          - Uniform surface (blank wall, calm water, featureless ground)
     *          - Poor lighting (too dark or too bright/washed out)
     *          - Surface too far (beyond sensor's effective range)
     *          - Motion blur (vehicle moving too fast)
     *          - Sensor obscured (dirt, condensation on lens)
     * 
     * @return uint8_t Surface quality 0-255 (0=invalid, 255=excellent)
     * 
     * @note EKF uses quality to weight optical flow measurements
     * @note Check quality() > threshold before trusting flow measurements
     * @see healthy() for overall sensor health status
     */
    uint8_t quality() const { return _state.surface_quality; }

    /**
     * @brief Get raw sensor flow rate
     * 
     * @details Returns the raw angular flow rate measured by the sensor in radians per
     *          second about the X and Y body axes. This is the unadjusted measurement
     *          before IMU gyro compensation.
     * 
     *          Coordinate Convention:
     *          - X-axis: Flow rate about body X-axis (forward), positive = right rotation
     *          - Y-axis: Flow rate about body Y-axis (right), positive = nose-down rotation
     *          - Right-hand rule: Thumb points along axis, fingers curl in positive direction
     * 
     *          Relationship to Velocity:
     *          velocity (m/s) = flowRate (rad/s) * height_above_ground (m)
     * 
     * @return const Vector2f& Flow rate in rad/s (X = roll axis, Y = pitch axis)
     * 
     * @note Units: radians per second (rad/s)
     * @note Frame: Body frame (X forward, Y right, Z down)
     * @note This is raw sensor output before gyro compensation
     * @see bodyRate() for gyro-compensated flow rate
     */
    const Vector2f& flowRate() const { return _state.flowRate; }

    /**
     * @brief Get gyro-compensated body rate
     * 
     * @details Returns the inertial body angular rate measured by the IMU in radians per
     *          second about the X and Y body axes. This represents the vehicle's rotation
     *          as measured by the gyroscope, which is used to compensate the raw flow
     *          measurement for vehicle rotation.
     * 
     *          Flow Compensation:
     *          The EKF uses both flowRate and bodyRate to separate vehicle motion from
     *          vehicle rotation:
     *              compensated_flow = flowRate - bodyRate
     * 
     *          This compensation removes the portion of optical flow caused by vehicle
     *          rotation, leaving only the flow caused by translational motion.
     * 
     * @return const Vector2f& Body inertial rate in rad/s (X = roll rate, Y = pitch rate)
     * 
     * @note Units: radians per second (rad/s)
     * @note Frame: Body frame (X forward, Y right, Z down)
     * @note Source: IMU gyroscope (not optical flow sensor)
     * @note Right-hand rule convention
     * @see flowRate() for raw sensor measurements
     */
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    /**
     * @brief Get timestamp of last sensor update
     * 
     * @details Returns the system time in milliseconds when the last optical flow
     *          measurement was received from the backend. Used to detect sensor
     *          timeouts and calculate measurement latency.
     * 
     *          Timeout Detection:
     *          If (millis() - last_update() > timeout_threshold), sensor may be
     *          disconnected or non-functional. Typical timeout threshold is 500-1000ms.
     * 
     * @return uint32_t Timestamp in milliseconds (from AP_HAL::millis())
     * 
     * @note Units: milliseconds
     * @note Wraps around every ~49.7 days (uint32_t overflow)
     * @note Compare with current millis() to determine age of measurement
     * @see healthy() for overall sensor health including timeout detection
     */
    uint32_t last_update() const { return _last_update_ms; }

    /**
     * @brief Get user-specified sensor height override
     * 
     * @details Returns the FLOW_HGT_OVR parameter value, which specifies a fixed height
     *          of the sensor above the ground. This parameter is primarily used on rovers
     *          where the sensor height is constant and a rangefinder may not be present.
     * 
     *          Height Usage:
     *          - If FLOW_HGT_OVR > 0: Use this fixed height value (meters)
     *          - If FLOW_HGT_OVR == 0: Use rangefinder measurement for height
     * 
     *          The EKF uses height to convert angular flow rate to linear velocity:
     *              velocity = flowRate * height
     * 
     * @return float Sensor height above ground in meters (0 = use rangefinder)
     * 
     * @note Units: meters
     * @note Range: 0-2 meters (0 = disabled, use rangefinder)
     * @note Primarily for rover applications with fixed sensor height
     * @warning Incorrect height value causes velocity estimation errors
     * @see flowRate() for the angular rate that is scaled by this height
     */
    float get_height_override() const { return _height_override; }

    /**
     * @struct OpticalFlow_state
     * @brief Optical flow measurement data structure
     * 
     * @details Contains a complete optical flow measurement including quality metric,
     *          raw sensor flow rate, and IMU body rate. This structure is used to
     *          pass measurements from backend to frontend via update_state(), and
     *          is then forwarded to the EKF for velocity estimation.
     * 
     *          Measurement Flow:
     *          1. Backend reads sensor and populates OpticalFlow_state
     *          2. Backend calls update_state() to push to frontend
     *          3. Frontend forwards to EKF via writeOptFlowMeas()
     *          4. EKF converts flow to velocity and fuses with other sensors
     * 
     * @note All fields must be populated by backend before calling update_state()
     * @note Coordinate frame: Body frame (X forward, Y right, Z down)
     */
    struct OpticalFlow_state {
        /**
         * @brief Image/surface quality metric
         * 
         * @details Surface quality as assessed by the optical flow sensor, ranging from
         *          0 (invalid/no features) to 255 (excellent quality/high confidence).
         *          Based on the contrast and texture of the surface below the sensor.
         * 
         * @note Units: dimensionless (0-255)
         * @note 0 = invalid, cannot trust flow measurements
         * @note 255 = excellent quality, high confidence
         */
        uint8_t  surface_quality;
        
        /**
         * @brief Optical flow angular rate about body axes
         * 
         * @details Angular flow rate in rad/s measured about the X and Y body axes.
         *          Represents the apparent motion of surface features across the sensor's
         *          field of view. A right-hand (RH) rotation about a sensor axis produces
         *          a positive rate.
         * 
         *          Conversion to Velocity:
         *          velocity = flowRate * height_above_ground
         * 
         * @note Units: radians per second (rad/s)
         * @note Frame: Body frame (X = forward/roll axis, Y = right/pitch axis)
         * @note Convention: Right-hand rule (positive = CW when looking along axis)
         */
        Vector2f flowRate;
        
        /**
         * @brief Body inertial angular rate from IMU
         * 
         * @details Inertial angular rate in rad/s measured by the IMU about the X and Y
         *          body axes. Used to compensate optical flow for vehicle rotation.
         *          A right-hand (RH) rotation about a sensor axis produces a positive rate.
         * 
         *          Flow Compensation:
         *          compensated_flow = flowRate - bodyRate
         * 
         * @note Units: radians per second (rad/s)
         * @note Frame: Body frame (X = forward/roll axis, Y = right/pitch axis)
         * @note Convention: Right-hand rule (positive = CW when looking along axis)
         * @note Source: IMU gyroscope, not optical flow sensor
         */
        Vector2f bodyRate;
    };

    /**
     * @brief Get sensor position offset from IMU/body frame origin
     * 
     * @details Returns a 3D vector defining the position offset of the optical flow
     *          sensor's focal point in meters relative to the body frame origin (IMU).
     *          This offset is critical for accurate EKF fusion as it accounts for
     *          the lever arm between the IMU and the optical flow sensor.
     * 
     *          Body Frame Convention:
     *          - X: Forward (positive = ahead of IMU)
     *          - Y: Right (positive = right of IMU)
     *          - Z: Down (positive = below IMU)
     * 
     *          Lever Arm Effect:
     *          When the vehicle rotates, the sensor position offset causes an apparent
     *          translation that must be accounted for in the EKF. The EKF uses this
     *          offset to correctly fuse optical flow with other sensors.
     * 
     *          Configuration:
     *          Set via FLOW_POS_X, FLOW_POS_Y, FLOW_POS_Z parameters (-5 to +5 meters each)
     * 
     * @return const Vector3f& Position offset in meters (body frame: X forward, Y right, Z down)
     * 
     * @note Units: meters
     * @note Frame: Body frame (NED convention)
     * @note Accuracy: Position offset should be accurate to ~1cm for best EKF performance
     * @warning Incorrect offset causes navigation errors, especially during aggressive maneuvers
     * @see _pos_offset parameter (FLOW_POS_X/Y/Z)
     */
    const Vector3f &get_pos_offset(void) const {
        return _pos_offset;
    }

    /**
     * @brief Begin automatic optical flow calibration
     * 
     * @details Starts the automatic calibration process for optical flow scaling factors.
     *          Calibration determines the FLOW_FXSCALER and FLOW_FYSCALER corrections
     *          by comparing optical flow velocity estimates with GPS velocity over a
     *          calibration flight.
     * 
     *          Calibration Process:
     *          1. Allocates calibrator object (_calibrator)
     *          2. Calls _calibrator->start() to begin data collection
     *          3. Collects flow and GPS data during flight
     *          4. Computes scaling corrections
     *          5. Updates FLOW_FXSCALER and FLOW_FYSCALER parameters
     * 
     *          Calibration Requirements:
     *          - GPS must be available and healthy
     *          - Vehicle must be in motion (flying/driving)
     *          - Good surface texture below sensor
     *          - Adequate lighting conditions
     * 
     * @note Only available when AP_OPTICALFLOW_CALIBRATOR_ENABLED
     * @note Calibration typically takes 30-60 seconds of flight
     * @warning Ensure safe flight conditions during calibration
     * @see stop_calibration() to halt calibration
     * @see _calibrator for calibration object
     */
    void start_calibration();
    
    /**
     * @brief Stop automatic optical flow calibration
     * 
     * @details Halts the automatic calibration process and deallocates the calibrator
     *          object. If calibration was in progress, computed scaling factors may
     *          be applied to FLOW_FXSCALER and FLOW_FYSCALER parameters.
     * 
     * @note Only available when AP_OPTICALFLOW_CALIBRATOR_ENABLED
     * @note Safe to call even if calibration is not active
     * @see start_calibration() to begin calibration
     */
    void stop_calibration();

    /**
     * @brief Parameter variable information table
     * 
     * @details Static table defining all AP_Param parameters for the optical flow subsystem.
     *          Used by AP_Param system for parameter storage, serialization, and ground
     *          station access.
     * 
     *          Registered Parameters:
     *          - FLOW_TYPE: Sensor type selection (Type enum)
     *          - FLOW_FXSCALER: X-axis scaling correction (parts per thousand)
     *          - FLOW_FYSCALER: Y-axis scaling correction (parts per thousand)
     *          - FLOW_ORIENT_YAW: Sensor yaw orientation (centidegrees)
     *          - FLOW_POS_X: Sensor X position offset (meters)
     *          - FLOW_POS_Y: Sensor Y position offset (meters)
     *          - FLOW_POS_Z: Sensor Z position offset (meters)
     *          - FLOW_ADDR: I2C address (0-127)
     *          - FLOW_HGT_OVR: Height override for rovers (meters)
     * 
     * @note Defined in AP_OpticalFlow.cpp
     * @see AP_Param::GroupInfo for table structure
     */
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Singleton instance pointer
     * 
     * @details Static pointer to the single AP_OpticalFlow instance. Set during
     *          construction and accessed via get_singleton() or AP::opticalflow().
     * 
     * @note Only one instance should exist per system
     * @see get_singleton()
     * @see AP::opticalflow()
     */
    static AP_OpticalFlow *_singleton;

    /**
     * @brief Pointer to active backend driver
     * 
     * @details Polymorphic pointer to the instantiated backend driver (OpticalFlow_backend
     *          subclass). Allocated during init() based on FLOW_TYPE parameter.
     *          nullptr if optical flow is disabled or backend allocation failed.
     * 
     *          Backend Lifecycle:
     *          1. init() calls backend detect() factory method
     *          2. If detection succeeds, allocate with NEW_NOTHROW
     *          3. Backend registers periodic update callback
     *          4. Backend deallocated on destruction
     * 
     * @note nullptr indicates no backend (FLOW_TYPE=NONE or allocation failure)
     * @see init() for backend instantiation
     */
    OpticalFlow_backend *backend;

    /**
     * @struct AP_OpticalFlow_Flags
     * @brief Status flags bitfield
     * 
     * @details Compact bitfield structure containing optical flow subsystem status flags.
     *          Uses bit packing to minimize memory footprint.
     */
    struct AP_OpticalFlow_Flags {
        /**
         * @brief Sensor health status flag
         * 
         * @details True if backend exists and is reporting healthy status (receiving
         *          valid measurements). False if backend is absent, not communicating,
         *          or reporting errors.
         * 
         *          Health Criteria:
         *          - Backend successfully instantiated
         *          - Recent measurements received (no timeout)
         *          - Communication with sensor functioning
         *          - Measurements passing backend validation
         * 
         * @note Set by backend via update_state()
         * @note Cleared on communication timeout or backend errors
         */
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    /**
     * @brief Optical flow sensor type parameter (FLOW_TYPE)
     * 
     * @details Selects which optical flow backend driver to instantiate during init().
     *          Corresponds to Type enum values. Changes require reboot to take effect.
     * 
     *          Common Values:
     *          - 0 = NONE (disabled)
     *          - 1 = PX4Flow I2C
     *          - 2 = Pixart SPI (PMW3900/3901)
     *          - 5 = MAVLink
     *          - 6 = DroneCAN/UAVCAN
     *          - 10 = SITL simulation
     * 
     * @note Type: AP_Enum<Type>
     * @note Default: NONE (0)
     * @note GCS Parameter: FLOW_TYPE
     * @see Type enum for available backend options
     */
    AP_Enum<Type>  _type;

    /**
     * @brief X-axis flow scaling correction parameter (FLOW_FXSCALER)
     * 
     * @details Scaling correction factor for X-axis flow measurements in parts per thousand.
     *          Used to compensate for sensor calibration errors, lens distortion, or
     *          field-of-view inaccuracies.
     * 
     *          Calculation:
     *          corrected_flow_x = raw_flow_x * (1.0 + FLOW_FXSCALER / 1000.0)
     * 
     *          Typical Calibration Results:
     *          - -200 to +200: Normal calibration corrections
     *          - Values beyond ±400: Indicates sensor or mounting problems
     * 
     * @note Type: AP_Int16
     * @note Range: -800 to +800 parts per thousand
     * @note Default: 0 (no correction, scaling = 1.0)
     * @note Units: parts per thousand (0 = 1.0x scaling)
     * @note GCS Parameter: FLOW_FXSCALER
     * @warning Large corrections (>±400) may indicate sensor issues
     * @see start_calibration() for automatic calibration
     */
    AP_Int16 _flowScalerX;

    /**
     * @brief Y-axis flow scaling correction parameter (FLOW_FYSCALER)
     * 
     * @details Scaling correction factor for Y-axis flow measurements in parts per thousand.
     *          Used to compensate for sensor calibration errors, lens distortion, or
     *          field-of-view inaccuracies.
     * 
     *          Calculation:
     *          corrected_flow_y = raw_flow_y * (1.0 + FLOW_FYSCALER / 1000.0)
     * 
     *          Typical Calibration Results:
     *          - -200 to +200: Normal calibration corrections
     *          - Values beyond ±400: Indicates sensor or mounting problems
     * 
     * @note Type: AP_Int16
     * @note Range: -800 to +800 parts per thousand
     * @note Default: 0 (no correction, scaling = 1.0)
     * @note Units: parts per thousand (0 = 1.0x scaling)
     * @note GCS Parameter: FLOW_FYSCALER
     * @warning Large corrections (>±400) may indicate sensor issues
     * @see start_calibration() for automatic calibration
     */
    AP_Int16 _flowScalerY;

    /**
     * @brief Sensor yaw orientation parameter (FLOW_ORIENT_YAW)
     * 
     * @details Yaw angle offset of the optical flow sensor's X-axis with respect to
     *          the vehicle's X-axis (forward direction) in centidegrees. Used when
     *          the sensor is mounted at a non-zero yaw angle.
     * 
     *          Rotation Convention:
     *          - 0° = Sensor X-axis aligned with vehicle forward
     *          - 90° = Sensor rotated 90° clockwise (looking down)
     *          - Positive = clockwise rotation when viewed from above
     * 
     *          Flow measurements are rotated by this angle before being used by the EKF:
     *          flow_vehicle_frame = rotate(flow_sensor_frame, -FLOW_ORIENT_YAW)
     * 
     * @note Type: AP_Int16
     * @note Range: -17999 to +18000 centidegrees (-179.99° to +180.00°)
     * @note Default: 0 (sensor aligned with vehicle)
     * @note Units: centidegrees (9000 = 90.00°)
     * @note GCS Parameter: FLOW_ORIENT_YAW
     * @warning Incorrect orientation causes navigation errors
     */
    AP_Int16 _yawAngle_cd;

    /**
     * @brief Sensor position offset parameter (FLOW_POS_X/Y/Z)
     * 
     * @details 3D position offset of the optical flow sensor's focal point in meters
     *          relative to the body frame origin (IMU location). Critical for accurate
     *          EKF fusion as it accounts for the lever arm between IMU and sensor.
     * 
     *          Body Frame Convention:
     *          - X: Forward (positive = ahead of IMU)
     *          - Y: Right (positive = right of IMU)
     *          - Z: Down (positive = below IMU)
     * 
     *          Measurement Procedure:
     *          1. Locate IMU position (body frame origin)
     *          2. Measure X: Forward distance from IMU to sensor (positive forward)
     *          3. Measure Y: Right distance from IMU to sensor (positive right)
     *          4. Measure Z: Down distance from IMU to sensor (positive down)
     * 
     *          Accuracy Requirements:
     *          - ±1cm accuracy recommended for best EKF performance
     *          - Errors cause navigation drift, especially during rotation
     * 
     * @note Type: AP_Vector3f
     * @note Range: -5.0 to +5.0 meters per axis
     * @note Default: (0, 0, 0) - sensor co-located with IMU
     * @note Units: meters
     * @note GCS Parameters: FLOW_POS_X, FLOW_POS_Y, FLOW_POS_Z
     * @warning Incorrect offset causes navigation errors during aggressive maneuvers
     * @see get_pos_offset() for accessor method
     */
    AP_Vector3f _pos_offset;

    /**
     * @brief I2C address parameter (FLOW_ADDR)
     * 
     * @details I2C bus address for I2C-based optical flow sensors (primarily PX4Flow).
     *          Allows selecting between multiple possible I2C addresses if sensor
     *          supports address configuration via hardware pins.
     * 
     *          Common Addresses:
     *          - 0x42 (66 decimal): PX4Flow default address
     *          - 0x43-0x49: Alternative addresses if multiple sensors on same bus
     * 
     * @note Type: AP_Int8
     * @note Range: 0-127 (valid I2C addresses)
     * @note Default: 0 (backend uses driver-specific default, typically 0x42 for PX4Flow)
     * @note GCS Parameter: FLOW_ADDR
     * @note Only applicable to I2C-based sensors
     * @see Type::PX4FLOW for primary use case
     */
    AP_Int8  _address;

    /**
     * @brief Sensor height override parameter (FLOW_HGT_OVR)
     * 
     * @details User-specified fixed height of the sensor above the ground in meters.
     *          Primarily used on rover applications where sensor height is constant
     *          and a rangefinder may not be available.
     * 
     *          Usage:
     *          - 0.0: Use rangefinder measurement for height (default behavior)
     *          - >0.0: Use this fixed height value instead of rangefinder
     * 
     *          The EKF converts angular flow rate to linear velocity using:
     *          velocity = flowRate * height
     * 
     *          Without accurate height, velocity estimation will be scaled incorrectly.
     * 
     * @note Type: AP_Float
     * @note Range: 0.0 to 2.0 meters
     * @note Default: 0.0 (use rangefinder)
     * @note Units: meters
     * @note GCS Parameter: FLOW_HGT_OVR
     * @note Primarily for rover applications with fixed sensor height
     * @warning Incorrect height causes proportional velocity estimation errors
     * @see get_height_override() for accessor method
     */
    AP_Float  _height_override;

    /**
     * @brief Backend callback to update frontend state
     * 
     * @details Called by backend drivers to push new optical flow measurements to the
     *          frontend. The backend populates an OpticalFlow_state structure with
     *          quality, flowRate, and bodyRate, then calls this method to transfer
     *          the data to the frontend.
     * 
     *          Update Sequence:
     *          1. Backend reads sensor and fills OpticalFlow_state
     *          2. Backend calls update_state(state)
     *          3. Frontend copies state to _state
     *          4. Frontend sets _flags.healthy = true
     *          5. Frontend updates _last_update_ms timestamp
     * 
     * @param[in] state Complete optical flow measurement structure containing quality,
     *                  flowRate, and bodyRate fields
     * 
     * @note Called by backend, not by application code
     * @note Sets healthy flag and updates timestamp
     * @note Thread-safety: Assumes single-threaded scheduler context
     * @see OpticalFlow_state for measurement structure
     */
    void update_state(const OpticalFlow_state &state);

    /**
     * @brief Current optical flow state
     * 
     * @details Stores the most recent optical flow measurement received from the backend.
     *          Updated by update_state() and accessed via quality(), flowRate(), and
     *          bodyRate() public methods.
     * 
     * @note Updated by backend via update_state()
     * @see update_state() for how this is populated
     * @see OpticalFlow_state for structure definition
     */
    struct OpticalFlow_state _state;

    /**
     * @brief Timestamp of last measurement update
     * 
     * @details System time in milliseconds (from AP_HAL::millis()) when the last optical
     *          flow measurement was received. Used for timeout detection and measurement
     *          age calculation.
     * 
     * @note Units: milliseconds
     * @note Updated by update_state()
     * @note Wraps around every ~49.7 days (uint32_t overflow)
     * @see last_update() for public accessor
     */
    uint32_t _last_update_ms;

    /**
     * @brief Write optical flow data to binary log
     * 
     * @details Writes an OF (Optical Flow) log message to the dataflash/SD card binary
     *          log when HAL_LOGGING_ENABLED. Log includes flowRate, bodyRate, and
     *          quality fields along with timestamp.
     * 
     *          Log Message Format (OF):
     *          - TimeUS: Timestamp in microseconds
     *          - Qual: Surface quality (0-255)
     *          - flowX: Flow rate X-axis (rad/s)
     *          - flowY: Flow rate Y-axis (rad/s)
     *          - bodyX: Body rate X-axis (rad/s)
     *          - bodyY: Body rate Y-axis (rad/s)
     * 
     *          Logging is conditional on _log_bit mask compared to LOG_BITMASK parameter.
     * 
     * @note Only available when HAL_LOGGING_ENABLED
     * @note Called from update() if logging is enabled
     * @note Logging frequency matches update() call rate (10-50Hz typically)
     * @see _log_bit for logging control
     */
    void Log_Write_Optflow();
    
    /**
     * @brief Logging control bitmask
     * 
     * @details Bitmask bit which indicates if optical flow data should be logged.
     *          Compared against the LOG_BITMASK parameter to determine if logging
     *          is currently enabled.
     * 
     *          Special Values:
     *          - -1 (0xFFFFFFFF): Always log regardless of LOG_BITMASK
     *          - Specific bit (e.g., LOG_OPTFLOW_BIT): Log only when that bit is set in LOG_BITMASK
     * 
     *          Set during init() based on the log_bit parameter.
     * 
     * @note Default: -1 (always log)
     * @note Type: uint32_t
     * @see init() for initialization
     * @see Log_Write_Optflow() for logging implementation
     */
    uint32_t _log_bit = -1;

#if AP_OPTICALFLOW_CALIBRATOR_ENABLED
    /**
     * @brief Optical flow calibrator object
     * 
     * @details Pointer to AP_OpticalFlow_Calibrator instance used for automatic
     *          calibration of flow scaling factors (FLOW_FXSCALER, FLOW_FYSCALER).
     *          Allocated when start_calibration() is called, deallocated when
     *          stop_calibration() is called or calibration completes.
     * 
     *          Calibration Process:
     *          - Collects optical flow and GPS velocity measurements during flight
     *          - Computes least-squares fit to determine scaling corrections
     *          - Updates FLOW_FXSCALER and FLOW_FYSCALER parameters
     * 
     * @note Only available when AP_OPTICALFLOW_CALIBRATOR_ENABLED
     * @note nullptr when calibration is not active
     * @note Allocated with NEW_NOTHROW, may fail if out of memory
     * @see start_calibration() for allocation and initialization
     * @see stop_calibration() for deallocation
     */
    AP_OpticalFlow_Calibrator *_calibrator;
#endif
};

/**
 * @namespace AP
 * @brief ArduPilot namespace for global singleton accessors
 */
namespace AP {
    /**
     * @brief Global namespace accessor for optical flow singleton
     * 
     * @details Convenience accessor for the AP_OpticalFlow singleton instance. Provides
     *          cleaner syntax compared to AP_OpticalFlow::get_singleton(). This is the
     *          preferred method for accessing optical flow functionality in vehicle code.
     * 
     *          Usage Example:
     *          @code
     *          // Check if optical flow is healthy
     *          if (AP::opticalflow() != nullptr && AP::opticalflow()->healthy()) {
     *              uint8_t quality = AP::opticalflow()->quality();
     *              Vector2f flow = AP::opticalflow()->flowRate();
     *          }
     *          @endcode
     * 
     * @return AP_OpticalFlow* Pointer to singleton instance, or nullptr if not initialized
     * 
     * @note Preferred accessor method for application code
     * @note Always check for nullptr before dereferencing
     * @see AP_OpticalFlow::get_singleton() for alternative accessor
     */
    AP_OpticalFlow *opticalflow();
}

#include "AP_OpticalFlow_Backend.h"

#endif // AP_OPTICALFLOW_ENABLED
