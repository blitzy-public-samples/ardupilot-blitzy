#pragma once

/**
 * @file AP_InertialSensor_NONE.h
 * @brief Null IMU backend for boards without physical sensors
 * 
 * @details This file implements a stub IMU backend for platforms without physical
 *          IMU hardware, typically embedded companion computers or peripheral CAN nodes.
 *          It provides a minimal functional IMU interface that allows the system to
 *          boot successfully and establish connectivity (WiFi, MAVLink) without actual
 *          sensor hardware.
 * 
 *          This implementation is derived from the SITL backend with all simulation
 *          logic removed and replaced with constant zero values.
 * 
 * @note Only compiled for ESP32 platform (CONFIG_HAL_BOARD == HAL_BOARD_ESP32)
 * @note Used primarily in AP_Periph firmware for peripheral nodes
 * 
 * @warning This backend provides no valid sensor data - for peripheral/development use only
 * @warning Flight control is impossible with this backend - vehicle will fail pre-arm checks
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @brief Default sensor sample rates for null IMU backend
 * 
 * @details Sample rates in Hz for gyro and accelerometer.
 *          Values match Pixhawk1 hardware (1000 Hz) for compatibility
 *          with vehicle code expecting these rates, even though no actual
 *          sampling occurs.
 * 
 *          Array format: [gyro_rate_hz, accel_rate_hz]
 */
const uint16_t INS_NONE_SENSOR_A[] = { 1000, 1000 };

/**
 * @class AP_InertialSensor_NONE
 * @brief Null backend for IMU-less platforms
 * 
 * @details Minimal IMU backend providing stub sensor interface for platforms
 *          without physical IMU hardware. This backend allows the ArduPilot
 *          system to initialize successfully on boards like ESP32 that serve
 *          as peripheral nodes or companion computers without requiring actual
 *          gyroscope or accelerometer hardware.
 *          
 *          Functionality provided:
 *          - Zero sensor readings (all axes return 0.0)
 *          - Proper backend registration for system initialization
 *          - Simulated update timing matching Pixhawk1 rates
 *          - No hardware initialization or polling
 *          - Always reports unhealthy status to prevent flight use
 *          
 *          Primary use cases:
 *          - ESP32 peripheral nodes (AP_Periph firmware)
 *          - Companion computers without IMU hardware
 *          - Development/testing environments without hardware
 *          - Firmware variants that don't require IMU (non-flight applications)
 *          
 *          Backend behavior:
 *          - get_gyro() returns Vector3f(0, 0, 0) for all axes
 *          - get_accel() returns Vector3f(0, 0, 0) for all axes
 *          - get_gyro_health() returns false (unhealthy status)
 *          - get_accel_health() returns false (unhealthy status)
 *          - update() performs timing management but generates no real data
 *          - Maintains sample timing to prevent scheduler issues
 *          
 *          Integration with vehicle code:
 *          - Automatically registered when no physical IMU detected on ESP32
 *          - Vehicle code must check IMU health before using data
 *          - Pre-arm checks will fail, preventing flight operations
 *          - Allows MAVLink communication and parameter management
 * 
 * @note Backend is automatically selected on ESP32 boards without IMU hardware
 * @note Inherits from AP_InertialSensor_Backend for standard IMU interface
 * @note Sample rates match Pixhawk1 (1000 Hz) for scheduler compatibility
 * 
 * @warning Flight control is impossible without valid IMU data
 * @warning This backend is for peripheral nodes and development only
 * @warning Vehicle will fail all pre-arm checks with this backend
 * @warning Do not use on flight-critical systems
 * 
 * @see AP_InertialSensor_Backend for base class interface
 * @see AP_InertialSensor for main sensor manager
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_NONE.h
 */
class AP_InertialSensor_NONE : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Constructor for null IMU backend
     * 
     * @details Initializes the null IMU backend with specified sample rates.
     *          Registers gyro and accelerometer instances with the main
     *          InertialSensor manager and configures timing parameters.
     *          No hardware initialization is performed.
     * 
     * @param[in] imu           Reference to main AP_InertialSensor manager
     * @param[in] sample_rates  Array of sample rates [gyro_hz, accel_hz]
     * 
     * @note Typically called by detect() method, not directly by vehicle code
     * @note Sample rates should match expected hardware rates for scheduler compatibility
     */
    AP_InertialSensor_NONE(AP_InertialSensor &imu, const uint16_t sample_rates[]);

    /**
     * @brief Update accelerometer and gyroscope state
     * 
     * @details Called by main scheduler loop to update sensor state.
     *          For null backend, this generates zero values for all axes
     *          and maintains timing to prevent scheduler issues.
     *          
     *          No actual sensor polling occurs. Backend generates:
     *          - Zero gyro rates on all axes (rad/s)
     *          - Zero accelerations on all axes (m/s²)
     *          - Maintains sample timestamps for scheduler
     * 
     * @return true if update successful (always true for null backend)
     * 
     * @note Called at main loop rate by AP_InertialSensor
     * @note Does not block or wait for hardware
     * 
     * @see AP_InertialSensor_Backend::update()
     */
    bool update() override;
    
    /**
     * @brief Start sensor data collection
     * 
     * @details Registers periodic timer callback for sensor updates.
     *          For null backend, this sets up timing infrastructure
     *          but does not initialize any hardware or start actual sampling.
     *          
     *          Registers timer_update() to be called at gyro sample rate
     *          to maintain scheduler timing expectations.
     * 
     * @note Called during system initialization by AP_InertialSensor
     * @note No hardware operations performed
     * 
     * @see timer_update()
     */
    void start() override;
    
    /**
     * @brief Accumulate sensor samples for current update period
     * 
     * @details Called by scheduler to accumulate sensor samples between updates.
     *          For null backend, this is a no-op as no actual sampling occurs.
     *          Method exists to satisfy backend interface requirements.
     * 
     * @note Does nothing for null backend
     * @note Required by AP_InertialSensor_Backend interface
     */
    void accumulate() override;
    
    /**
     * @brief Detect and instantiate null IMU backend
     * 
     * @details Factory method to create null IMU backend instance.
     *          Always succeeds as no hardware detection is required.
     *          Creates backend with specified sample rates and returns
     *          pointer for registration with main sensor manager.
     *          
     *          Detection logic:
     *          - No hardware probing performed
     *          - Always returns valid backend instance
     *          - Used when no physical IMU hardware present
     *          
     *          Sample rate configuration:
     *          - Typically INS_NONE_SENSOR_A (1000 Hz gyro/accel)
     *          - Rates must match scheduler expectations
     * 
     * @param[in] imu           Reference to main AP_InertialSensor manager
     * @param[in] sample_rates  Array of sample rates [gyro_hz, accel_hz]
     * 
     * @return Pointer to new backend instance (never nullptr for null backend)
     * 
     * @note Called by AP_InertialSensor during backend detection phase
     * @note Returns heap-allocated backend (managed by InertialSensor)
     * 
     * @see AP_InertialSensor::_add_backend()
     */
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu, const uint16_t sample_rates[]);

private:
    /**
     * @brief Initialize sensor hardware (no-op for null backend)
     * @return true always (no hardware to initialize)
     */
    bool init_sensor(void);
    
    /**
     * @brief Timer callback for periodic sensor updates
     * @details Generates zero-value samples at configured rates and
     *          publishes to sensor manager with current timestamps
     */
    void timer_update();
    
    /**
     * @brief Calculate gyro drift (returns 0.0 for null backend)
     * @return float Gyro drift in rad/s (always 0.0)
     */
    float gyro_drift(void);
    
    /**
     * @brief Generate simulated accelerometer data (zeros)
     * @details Publishes zero acceleration samples to all axes
     */
    void generate_accel();
    
    /**
     * @brief Generate simulated gyroscope data (zeros)
     * @details Publishes zero rate samples to all axes
     */
    void generate_gyro();
    
    // Unused random float generator (commented out)

    // Legacy SITL pointer (unused in null backend)
    //SITL::SITL *sitl;

    const uint16_t gyro_sample_hz;      ///< Gyroscope sample rate in Hz (typically 1000)
    const uint16_t accel_sample_hz;     ///< Accelerometer sample rate in Hz (typically 1000)

    uint8_t gyro_instance;              ///< Gyro instance ID registered with sensor manager
    uint8_t accel_instance;             ///< Accel instance ID registered with sensor manager
    uint64_t next_gyro_sample;          ///< Timestamp for next gyro sample (microseconds)
    uint64_t next_accel_sample;         ///< Timestamp for next accel sample (microseconds)
    float gyro_time;                    ///< Accumulated gyro sample time (seconds)
    float accel_time;                   ///< Accumulated accel sample time (seconds)
    float gyro_motor_phase[32];         ///< Motor phase tracking for gyro (unused, legacy from SITL)
    float accel_motor_phase[32];        ///< Motor phase tracking for accel (unused, legacy from SITL)

    static uint8_t bus_id;              ///< Static bus ID for backend identification
};
#endif // CONFIG_HAL_BOARD
