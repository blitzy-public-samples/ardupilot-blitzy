/**
 * @file AP_HAL_Empty_Namespace.h
 * @brief Forward declarations for Empty HAL stub implementation classes
 * 
 * Provides lightweight forward declarations of all Empty HAL stub classes within
 * the Empty namespace. Used to minimize compilation dependencies and enable
 * forward references between Empty HAL components.
 * 
 * @note This is a template/stub implementation - not for actual hardware use
 * @note All classes provide minimal no-op or deterministic implementations
 */

#pragma once

/**
 * @namespace Empty
 * @brief Namespace containing stub HAL implementations for template and testing
 * 
 * @details The Empty namespace encapsulates all hardware abstraction stub classes
 *          that provide compile-time substitutes for actual hardware drivers.
 *          
 *          Purpose:
 *          - **Porting template**: Starting point for new platform HAL implementations
 *          - **CI builds**: Enable compilation without platform-specific hardware
 *          - **Unit testing**: Provides predictable, deterministic behavior
 *          - **Documentation**: Reference showing required interface surface
 *          
 *          Implementation pattern:
 *          - All methods implemented (no pure virtuals - links successfully)
 *          - Minimal or no-op implementations (no actual hardware access)
 *          - Deterministic return values (e.g., AnalogIn always returns 1.11)
 *          - No thread safety (intended for single-threaded host builds)
 *          
 *          Forward-declared classes correspond 1:1 with AP_HAL interfaces:
 *          - AnalogIn, AnalogSource: ADC and voltage monitoring stubs
 *          - GPIO, DigitalSource: Digital I/O stubs (no actual pin control)
 *          - UARTDriver: Serial port stub (write succeeds, read returns 0)
 *          - SPIDevice, SPIDeviceDriver, SPIDeviceManager: SPI bus stubs
 *          - I2CDevice, I2CDeviceManager: I2C bus stubs
 *          - WSPIDevice, WSPIDeviceManager: Wide SPI (Quad/Octo) stubs
 *          - RCInput, RCOutput: RC receiver and motor output stubs
 *          - Scheduler: Timing and task scheduling stubs (delays are no-ops)
 *          - Semaphore: Synchronization stub (simple boolean, not thread-safe)
 *          - Storage: Parameter persistence stub (read returns zeros, write is no-op)
 *          - Util: Utility functions stub
 *          - OpticalFlow: Optical flow sensor stub
 *          - Flash: Internal flash programming stub
 *          - DSP: Digital signal processing stub (no actual FFT)
 * 
 * @note Classes defined in individual headers (AnalogIn.h, GPIO.h, etc.)
 * @note Use AP_HAL_Empty_Private.h to include all implementation headers
 * @warning Not suitable for actual flight hardware - stubs only
 * @warning Not thread-safe - designed for single-threaded unit tests and host tools
 */
namespace Empty {
    class AnalogIn; /**< ADC manager stub - always returns fixed voltage values (1.11, 5.0V) */
    class AnalogSource; /**< Single ADC channel stub - provides deterministic readings */
    class DigitalSource; /**< Digital I/O pin stub with in-memory state storage */
    class DSP; /**< Digital Signal Processing stub - no actual FFT computation */
    class GPIO; /**< Digital I/O manager stub - read returns 0, write is no-op */
    class I2CDevice; /**< I2C device stub - transfers succeed without actual bus communication */
    class I2CDeviceManager; /**< I2C bus manager stub - returns nullptr for all devices */
    class OpticalFlow; /**< Optical flow sensor stub - no flow data available */
    class RCInput; /**< RC receiver input stub - returns default channel values (1500μs) */
    class RCOutput; /**< Motor/servo PWM output stub - stores values in memory array */
    class Scheduler; /**< Task scheduler stub - delays are no-ops, timing functions work */
    class Semaphore; /**< Mutex stub - simple boolean flag, always succeeds, not thread-safe */
    class SPIDevice; /**< SPI device stub - transfers succeed without actual bus communication */
    class SPIDeviceDriver; /**< SPI device driver stub for legacy compatibility */
    class SPIDeviceManager; /**< SPI bus manager stub - device allocation succeeds with stub devices */
    class WSPIDevice; /**< Wide SPI (Quad/Octo) device stub - no actual transfers */
    class WSPIDeviceManager; /**< Wide SPI manager stub - device allocation returns nullptr */
    class Storage; /**< Parameter storage stub - reads return zeros, writes are no-ops */
    class UARTDriver; /**< Serial port stub - write reports success, read returns no data */
    class Util; /**< Utility functions stub - minimal implementations with safe defaults */
    class Flash; /**< Internal flash programming stub - all operations return success without actual flash access */
}
