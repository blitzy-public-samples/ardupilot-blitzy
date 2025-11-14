/**
 * @file AP_HAL_SITL_Namespace.h
 * @brief Forward declarations for SITL (Software In The Loop) HAL implementation
 * 
 * @details This header provides namespace and forward declarations for all
 *          SITL-specific Hardware Abstraction Layer (HAL) component classes.
 *          These classes implement the AP_HAL interface for software simulation,
 *          enabling autopilot code to run on desktop/server environments without
 *          physical hardware.
 *          
 *          The SITL HAL is used for:
 *          - Development and testing without hardware
 *          - Automated testing in CI/CD pipelines
 *          - Algorithm development and debugging
 *          - Training and demonstration
 *          
 *          Each class declared here provides a simulation-specific implementation
 *          of the corresponding AP_HAL interface, with behavior tailored for
 *          software-in-the-loop simulation.
 * 
 * @see libraries/AP_HAL/AP_HAL_Namespace.h for the abstract HAL interfaces
 * @see libraries/AP_HAL_SITL/README.md for SITL architecture documentation
 */

#pragma once

/**
 * @namespace HALSITL
 * @brief Namespace for Software In The Loop (SITL) HAL implementation
 * 
 * @details The HALSITL namespace contains all simulation-specific implementations
 *          of the AP_HAL (ArduPilot Hardware Abstraction Layer) interfaces.
 *          These classes provide virtual hardware implementations that enable
 *          the autopilot firmware to run on standard operating systems (Linux,
 *          Windows, macOS) without physical hardware.
 *          
 *          Key characteristics of SITL HAL implementations:
 *          - Simulated sensors with configurable noise and failure modes
 *          - Virtual communication interfaces (UART, I2C, SPI, CAN)
 *          - Network-based RC input and servo output
 *          - Integration with physics simulators (JSBSim, Gazebo, etc.)
 *          - Timing control for deterministic testing
 *          - Storage backed by filesystem instead of flash/EEPROM
 *          
 *          The SITL HAL allows the exact same autopilot code that runs on
 *          embedded hardware to execute in simulation, ensuring high fidelity
 *          between simulated and real flight behavior.
 *          
 * @note All classes in this namespace are forward-declared here and fully
 *       defined in their respective header files within AP_HAL_SITL/
 */
namespace HALSITL {
class UARTDriver;
class Scheduler;
class SITL_State_Common;
class SITL_State;
class Storage;
class AnalogIn;
class I2CDevice;
class I2CDeviceManager;
class RCInput;
class RCOutput;
class ADCSource;
class RCInput;
class Util;
class Semaphore;
class BinarySemaphore;
class GPIO;
class DigitalSource;
class DSP;
class CANIface;
}  // namespace HALSITL
