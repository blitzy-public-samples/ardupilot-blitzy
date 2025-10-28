/**
 * @file HAL.h
 * @brief Main HAL class aggregating all hardware abstraction interfaces
 * 
 * @details Defines the central HAL class that aggregates pointers to all hardware interfaces
 *          (UART, SPI, I2C, GPIO, Scheduler, etc.) and provides the main entry point (run method)
 *          for firmware execution. Platform implementations (ChibiOS, Linux, SITL, ESP32) populate
 *          this structure with concrete drivers for their specific hardware.
 *          
 *          The HAL serves as the primary abstraction layer between portable ArduPilot firmware
 *          and platform-specific hardware drivers, enabling the same vehicle code to run on
 *          diverse hardware platforms.
 * 
 * @note This is a core ArduPilot HAL file - modifications affect all platforms
 * @see AP_HAL_ChibiOS for embedded ARM implementation
 * @see AP_HAL_Linux for Linux-based boards
 * @see AP_HAL_SITL for software simulation
 * @see AP_HAL_ESP32 for ESP32 platform
 */
#pragma once

class AP_Param;

#include "AP_HAL_Namespace.h"

#include "AnalogIn.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "SPIDevice.h"
#include "WSPIDevice.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "system.h"
#include "OpticalFlow.h"
#include "DSP.h"
#include "CANIface.h"

/**
 * @class AP_HAL::HAL
 * @brief Central hardware abstraction layer aggregator
 * 
 * @details The HAL class is the primary interface between ArduPilot firmware and
 *          platform-specific hardware drivers. It aggregates pointers to all
 *          hardware subsystem interfaces, providing a unified access point for
 *          vehicle code to interact with hardware without platform dependencies.
 *          
 *          **Hardware Subsystem Interfaces**:
 *          - **console**: Debug/USB serial console (typically USB or Serial0)
 *          - **serial[]**: Array of UART ports for telemetry, GPS, and peripherals
 *          - **i2c_mgr**: I2C bus manager for I2C sensor and peripheral access
 *          - **spi**: SPI bus manager for high-speed sensors (IMU, baro, etc.)
 *          - **wspi**: Wide SPI (Quad/Octo SPI) for high-speed flash memory
 *          - **analogin**: Analog-to-digital converter manager for voltage/current sensing
 *          - **storage**: Persistent parameter storage (EEPROM/Flash abstraction)
 *          - **gpio**: Digital I/O pin control
 *          - **rcin**: RC receiver input decoder
 *          - **rcout**: Motor/servo PWM output generator
 *          - **scheduler**: Task scheduling and timing services
 *          - **util**: Miscellaneous utilities (system info, safety state, etc.)
 *          - **opticalflow**: Optical flow sensor interface (optional)
 *          - **flash**: Internal flash memory programming interface (optional)
 *          - **dsp**: DSP operations including FFT (optional, HAL_WITH_DSP)
 *          - **can[]**: CAN bus interface array (optional, HAL_NUM_CAN_IFACES)
 *          
 *          **Platform Implementations**:
 *          - **ChibiOS**: Embedded ARM boards (Pixhawk, CubeOrange, Kakute, Matek, etc.)
 *          - **Linux**: Linux-based flight controllers (Navio, BeagleBone Blue, RPi)
 *          - **SITL**: Software-In-The-Loop simulation (desktop/CI testing)
 *          - **ESP32**: ESP32-based boards (ESP32-S3, etc.)
 *          - **QURT**: Qualcomm Hexagon DSP platform
 *          
 *          **Singleton Access Pattern**:
 *          The global `hal` object is defined by each platform implementation and
 *          provides singleton access to all hardware interfaces:
 *          ```cpp
 *          hal.console->printf("Boot complete\n");
 *          hal.scheduler->delay(100);
 *          hal.serial(0)->write(buffer, len);
 *          ```
 *          
 *          **Lifecycle**:
 *          1. Platform creates HAL object with concrete driver pointers (static initialization)
 *          2. HAL constructor calls AP_HAL::init() to initialize namespace globals
 *          3. Platform calls hal.run() to transfer control to firmware
 *          4. Firmware setup() and loop() callbacks execute via scheduler
 *          
 *          **Thread Safety**:
 *          - HAL object construction: Single-threaded (startup only)
 *          - HAL member access: Thread-safe after construction completes
 *          - Individual interface thread-safety: Documented per interface
 * 
 * @note HAL object is constructed once per platform during static initialization
 * @note All interface pointers must be non-null after init (except optional features)
 * @warning Accessing HAL members before construction completes causes undefined behavior
 * @warning Never delete or reassign HAL interface pointers - they are constant after init
 * 
 * @see UARTDriver for serial port interface documentation
 * @see Scheduler for task scheduling interface documentation
 * @see GPIO for digital I/O interface documentation
 * @see SPIDevice for SPI bus interface documentation
 * @see I2CDevice for I2C bus interface documentation
 */

class AP_HAL::HAL {
public:
    /**
     * @brief Construct HAL object with platform-specific driver implementations
     * 
     * @details Platform implementations call this constructor during static initialization
     *          to aggregate all hardware driver pointers into the central HAL object.
     *          The constructor initializes the serial port array and CAN interface array,
     *          then calls AP_HAL::init() to set up namespace-level globals.
     *          
     *          Serial port typical assignments:
     *          - serial0: Console/USB (also accessible via 'console' pointer)
     *          - serial1: Primary telemetry (typically MAVLink to GCS)
     *          - serial2: Secondary telemetry
     *          - serial3: Primary GPS
     *          - serial4: Secondary GPS
     *          - serial5-9: Additional peripherals (rangefinder, gimbal, etc.)
     *          
     *          Actual port usage configured via SERIALn_PROTOCOL parameters at runtime.
     * 
     * @param[in] _serial0     Serial port 0 - typically USB console
     * @param[in] _serial1     Serial port 1 - typically primary telemetry
     * @param[in] _serial2     Serial port 2 - typically secondary telemetry
     * @param[in] _serial3     Serial port 3 - typically primary GPS
     * @param[in] _serial4     Serial port 4 - typically secondary GPS
     * @param[in] _serial5     Serial port 5 - extra peripheral 1
     * @param[in] _serial6     Serial port 6 - extra peripheral 2
     * @param[in] _serial7     Serial port 7 - extra peripheral 3
     * @param[in] _serial8     Serial port 8 - extra peripheral 4
     * @param[in] _serial9     Serial port 9 - extra peripheral 5
     * @param[in] _i2c_mgr     I2C device manager instance
     * @param[in] _spi         SPI device manager instance
     * @param[in] _wspi        Wide SPI device manager instance (Quad/Octo SPI)
     * @param[in] _analogin    Analog input manager instance
     * @param[in] _storage     Persistent storage manager instance
     * @param[in] _console     Console UART (typically same as _serial0)
     * @param[in] _gpio        GPIO manager instance
     * @param[in] _rcin        RC input manager instance
     * @param[in] _rcout       RC/motor output manager instance
     * @param[in] _scheduler   Task scheduler instance
     * @param[in] _util        Utility functions instance
     * @param[in] _opticalflow Optical flow sensor interface (may be nullptr)
     * @param[in] _flash       Internal flash programming interface (may be nullptr)
     * @param[in] _simstate    Simulator state (SIM builds only)
     * @param[in] _dsp         DSP/FFT interface (HAL_WITH_DSP only, may be nullptr)
     * @param[in] _can_ifaces  Array of CAN interface pointers (HAL_NUM_CAN_IFACES)
     * 
     * @note Constructor never fails - platform must provide valid pointers
     * @note Optional interfaces (opticalflow, flash, dsp) may be nullptr
     * @note Called once during static initialization - never call directly
     * 
     * @warning All non-optional pointers must be valid for HAL lifetime
     * @warning Constructor must complete before any HAL member access
     */
    HAL(AP_HAL::UARTDriver* _serial0, // console
        AP_HAL::UARTDriver* _serial1, // telem1
        AP_HAL::UARTDriver* _serial2, // telem2
        AP_HAL::UARTDriver* _serial3, // 1st GPS
        AP_HAL::UARTDriver* _serial4, // 2nd GPS
        AP_HAL::UARTDriver* _serial5, // extra1
        AP_HAL::UARTDriver* _serial6, // extra2
        AP_HAL::UARTDriver* _serial7, // extra3
        AP_HAL::UARTDriver* _serial8, // extra4
        AP_HAL::UARTDriver* _serial9, // extra5
        AP_HAL::I2CDeviceManager* _i2c_mgr,
        AP_HAL::SPIDeviceManager* _spi,
        AP_HAL::WSPIDeviceManager* _wspi,
        AP_HAL::AnalogIn*   _analogin,
        AP_HAL::Storage*    _storage,
        AP_HAL::UARTDriver* _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler,
        AP_HAL::Util*       _util,
        AP_HAL::OpticalFlow*_opticalflow,
        AP_HAL::Flash*      _flash,
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        class AP_HAL::SIMState*   _simstate,
#endif
#if HAL_WITH_DSP
        AP_HAL::DSP*        _dsp,
#endif
#if HAL_NUM_CAN_IFACES > 0
        AP_HAL::CANIface* _can_ifaces[HAL_NUM_CAN_IFACES])
#else
        AP_HAL::CANIface** _can_ifaces)
#endif
        :
        i2c_mgr(_i2c_mgr),
        spi(_spi),
        wspi(_wspi),
        analogin(_analogin),
        storage(_storage),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler),
        util(_util),
        opticalflow(_opticalflow),
        flash(_flash),
#if HAL_WITH_DSP
        dsp(_dsp),
#endif
        serial_array{
            _serial0,
            _serial1,
            _serial2,
            _serial3,
            _serial4,
            _serial5,
            _serial6,
            _serial7,
            _serial8,
            _serial9}
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
            ,simstate(_simstate)
#endif
    {
#if HAL_NUM_CAN_IFACES > 0
        if (_can_ifaces == nullptr) {
            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
                can[i] = nullptr;
        } else {
            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
                can[i] = _can_ifaces[i];
        }
#endif

        AP_HAL::init();
    }

    /**
     * @struct Callbacks
     * @brief Firmware entry point interface called by HAL run loop
     * 
     * @details Vehicle firmware implements this interface to provide setup() and loop()
     *          functions that constitute the main firmware execution. The HAL run() method
     *          calls setup() once during initialization, then repeatedly calls loop() at
     *          the scheduler's main loop rate.
     *          
     *          Typical vehicle implementation pattern:
     *          ```cpp
     *          class Copter : public AP_HAL::HAL::Callbacks {
     *          public:
     *              void setup() override {
     *                  // One-time initialization
     *                  // Load parameters, initialize sensors, calibrate, etc.
     *                  // Must complete in bounded time (typically <5 seconds)
     *              }
     *              
     *              void loop() override {
     *                  // Main loop - called repeatedly
     *                  // Read sensors, run control loops, update outputs
     *                  // Must complete before next scheduler tick
     *                  // Rate determined by SCHED_LOOP_RATE (e.g., 400Hz)
     *              }
     *          };
     *          
     *          Copter copter;
     *          hal.run(argc, argv, &copter);  // Never returns
     *          ```
     *          
     *          Execution flow:
     *          1. Platform calls hal.run() with pointer to Callbacks implementation
     *          2. HAL initializes scheduler and hardware
     *          3. HAL calls callbacks->setup() once
     *          4. HAL enters infinite loop calling callbacks->loop() at scheduled rate
     *          5. Scheduler manages timing, preemption, and task dispatch
     * 
     * @note Pure virtual interface - vehicle code must implement both methods
     * @note setup() called once, loop() called repeatedly
     * @note setup() should complete initialization in bounded time (<5s recommended)
     * @note loop() must not block or delay - scheduler handles timing
     * 
     * @warning setup() blocking forever prevents firmware startup
     * @warning loop() overruns (execution time > loop period) cause scheduler warnings
     * @warning Exceptions in setup()/loop() are not caught - will crash firmware
     * 
     * @see FunCallbacks for function pointer wrapper utility
     * @see Scheduler for task scheduling and timing documentation
     */
    struct Callbacks {
        virtual void setup() = 0;
        virtual void loop() = 0;
    };

    /**
     * @struct FunCallbacks
     * @brief Utility class wrapping function pointers into Callbacks interface
     * 
     * @details Provides a convenience wrapper for using C-style function pointers
     *          with the Callbacks interface, avoiding the need to create a full class
     *          for simple test programs or examples.
     *          
     *          Typical usage in test programs:
     *          ```cpp
     *          void my_setup() {
     *              hal.console->printf("Setup running\n");
     *              // Initialization code
     *          }
     *          
     *          void my_loop() {
     *              hal.console->printf("Loop tick\n");
     *              hal.scheduler->delay(100);
     *          }
     *          
     *          AP_HAL::HAL::FunCallbacks callbacks(my_setup, my_loop);
     *          hal.run(argc, argv, &callbacks);
     *          ```
     *          
     *          This is primarily used in:
     *          - Simple test programs in libraries/*/examples/
     *          - Hardware bringup and testing
     *          - Minimal SITL test cases
     *          
     *          Production vehicle code typically inherits from Callbacks directly
     *          to access member variables and methods.
     * 
     * @note Useful for simple test programs and examples
     * @note Function pointers stored by value - must remain valid for HAL lifetime
     * @note No state management - use class inheritance for complex firmware
     * 
     * @see Callbacks for the base interface documentation
     */
    struct FunCallbacks : public Callbacks {
        /**
         * @brief Construct FunCallbacks with function pointers for setup and loop
         * 
         * @param[in] setup_fun Function pointer called once during setup phase
         * @param[in] loop_fun  Function pointer called repeatedly in main loop
         * 
         * @note Function pointers must remain valid for entire firmware execution
         * @note Functions are called directly with no additional state or context
         */
        FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void));

        /** @brief Override calling stored setup function pointer */
        void setup() override { _setup(); }
        
        /** @brief Override calling stored loop function pointer */
        void loop() override { _loop(); }

    private:
        void (*_setup)(void);  ///< Stored setup function pointer
        void (*_loop)(void);   ///< Stored loop function pointer
    };

    /**
     * @brief Pure virtual run method - platform main loop entry point
     * 
     * @details This is the main entry point where platform implementations transfer control
     *          to the firmware. Each platform (ChibiOS, Linux, SITL, ESP32) provides its
     *          own implementation that:
     *          
     *          1. Completes platform-specific hardware initialization
     *          2. Initializes the scheduler and timing subsystems
     *          3. Calls callbacks->setup() once for firmware initialization
     *          4. Enters infinite loop calling callbacks->loop() at scheduled rate
     *          5. Never returns (firmware runs until power off or reset)
     *          
     *          Platform-specific responsibilities:
     *          - **ChibiOS**: Start ChibiOS kernel, create threads, setup interrupts
     *          - **Linux**: Setup signal handlers, configure process priority
     *          - **SITL**: Initialize simulation physics, setup networking
     *          - **ESP32**: Configure FreeRTOS tasks, setup WiFi/Bluetooth
     *          
     *          Command-line argument handling:
     *          - Embedded platforms (ChibiOS): argc=0, argv=nullptr (no CLI)
     *          - SITL/Linux: argc/argv contain command-line options for configuration
     *          
     *          Example SITL invocation:
     *          ```
     *          ./arducopter --model quad --home 35.0,-118.0,0,0
     *          ```
     * 
     * @param[in] argc      Command-line argument count (0 for embedded platforms)
     * @param[in] argv      Command-line argument array (nullptr for embedded platforms)
     * @param[in] callbacks Pointer to firmware Callbacks implementation (setup/loop)
     * 
     * @note This method never returns - enters infinite firmware execution loop
     * @note Called once per firmware boot from platform main() function
     * @note Platform must complete all hardware init before calling callbacks->setup()
     * @note Scheduler initialized before entering loop() - timing services available
     * 
     * @warning Must never return - firmware expects to run indefinitely
     * @warning callbacks pointer must remain valid for entire firmware lifetime
     * @warning Exceptions or crashes in this method are fatal (no recovery)
     * 
     * @see Callbacks for firmware entry point interface documentation
     * @see Scheduler for main loop timing and task scheduling
     */
    virtual void run(int argc, char * const argv[], Callbacks* callbacks) const = 0;

public:
    /// I2C device manager - provides access to I2C buses for sensor communication
    AP_HAL::I2CDeviceManager* i2c_mgr;
    
    /// SPI device manager - provides access to SPI buses for high-speed sensors (IMU, barometer)
    AP_HAL::SPIDeviceManager* spi;
    
    /// Wide SPI device manager - Quad/Octo SPI for high-speed flash memory and storage
    AP_HAL::WSPIDeviceManager* wspi;
    
    /// Analog input manager - ADC interface for voltage, current, and analog sensor readings
    AP_HAL::AnalogIn*   analogin;
    
    /// Persistent storage - parameter and configuration data storage (EEPROM/Flash abstraction)
    AP_HAL::Storage*    storage;
    
    /// Console UART - primary debug/USB serial console (typically Serial0/USB)
    AP_HAL::UARTDriver* console;
    
    /// GPIO manager - digital I/O pin control for relays, LEDs, and general-purpose pins
    AP_HAL::GPIO*       gpio;
    
    /// RC input - RC receiver decoder for PPM, SBUS, DSM, and other RC protocols
    AP_HAL::RCInput*    rcin;
    
    /// RC output - PWM output generator for motors, servos, and other PWM devices
    AP_HAL::RCOutput*   rcout;
    
    /// Task scheduler - manages task timing, priorities, and main loop execution
    AP_HAL::Scheduler*  scheduler;
    
    /// Utility functions - system info, safety state, tone generation, and misc utilities
    AP_HAL::Util        *util;
    
    /// Optical flow sensor interface - provides velocity estimation from optical flow (optional)
    AP_HAL::OpticalFlow *opticalflow;
    
    /// Internal flash programming - allows firmware to read/write internal MCU flash (optional)
    AP_HAL::Flash       *flash;
    
    /// DSP interface - provides FFT and signal processing operations (optional, HAL_WITH_DSP)
    AP_HAL::DSP         *dsp;
    
#if HAL_NUM_CAN_IFACES > 0
    /// CAN bus interface array - DroneCAN/UAVCAN support for CAN peripherals and sensors
    AP_HAL::CANIface* can[HAL_NUM_CAN_IFACES];
#else
    /// CAN bus interface array - placeholder when no CAN support compiled
    AP_HAL::CANIface** can;
#endif

    /**
     * @brief Access serial ports using SERIALn_ numbering
     * 
     * @details Provides safe indexed access to UART drivers using the SERIAL0-SERIAL9
     *          numbering scheme. Returns nullptr for invalid port numbers.
     *          
     *          Port number mapping:
     *          - 0: Console/USB (same as console pointer)
     *          - 1: Primary telemetry (typically MAVLink to GCS)
     *          - 2: Secondary telemetry
     *          - 3: Primary GPS
     *          - 4: Secondary GPS
     *          - 5-9: Additional peripherals
     *          
     *          Actual usage determined by SERIALn_PROTOCOL parameters at runtime.
     *          
     *          Example usage:
     *          ```cpp
     *          UARTDriver* gps_port = hal.serial(3);
     *          if (gps_port) {
     *              gps_port->write(buffer, len);
     *          }
     *          ```
     * 
     * @param[in] sernum Serial port number (0-9)
     * 
     * @return Pointer to UARTDriver for requested port, nullptr if sernum >= num_serial
     * 
     * @note Prefer this method over direct serial_array access
     * @note Check return value for nullptr before using
     * @note Port function configured via SERIALn_PROTOCOL parameter at runtime
     * 
     * @see UARTDriver for UART interface documentation
     */
    UARTDriver* serial(uint8_t sernum) const;

    /// Number of serial ports in serial_array (SERIAL0 through SERIAL9)
    static constexpr uint8_t num_serial = 10;

private:
    /// UART driver array in SERIALn_ order (indexed 0-9 for SERIAL0-SERIAL9)
    AP_HAL::UARTDriver* serial_array[num_serial];

public:
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
    /// Simulator state interface - provides access to simulation state (SIM builds only)
    AP_HAL::SIMState *simstate;
#endif

/**
 * @def DEV_PRINTF
 * @brief Convenience macro for printing debug messages to console
 * 
 * @details Provides a simple printf-style interface for debug output to the HAL console.
 *          Automatically disabled when HAL_CONSOLE_DISABLED is defined (typically for
 *          minimal bootloader builds or production releases where console is not available).
 *          
 *          Example usage:
 *          ```cpp
 *          DEV_PRINTF("Sensor initialized: %d\n", sensor_id);
 *          DEV_PRINTF("GPS fix quality: %d satellites\n", num_sats);
 *          ```
 *          
 *          Expands to:
 *          - Normal builds: `hal.console->printf(fmt, ## args)`
 *          - Console disabled: Empty (no code generated)
 * 
 * @param[in] fmt  Printf-style format string
 * @param[in] args Variable arguments matching format string
 * 
 * @note Output goes to console UART (typically USB or Serial0)
 * @note Automatically disabled in builds without console support
 * @note Use for debug messages only - not for user-facing output
 * @note Consider using AP_Logger for permanent diagnostic data
 * 
 * @warning Do not use in performance-critical paths - printf is slow
 * @warning Do not rely on this for critical error messages - may be disabled
 * 
 * @see UARTDriver::printf for underlying implementation
 */
#ifndef HAL_CONSOLE_DISABLED
# define DEV_PRINTF(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while(0)
#else
# define DEV_PRINTF(fmt, args ...)
#endif

};

/**
 * @brief Inline implementation of serial port accessor
 * 
 * @details Returns pointer to requested serial port with bounds checking.
 *          Inline for performance since this is called frequently throughout
 *          the firmware for GPS, telemetry, and peripheral communication.
 * 
 * @param[in] sernum Serial port index (0-9)
 * @return UARTDriver pointer or nullptr if index out of range
 * 
 * @note Inlined for performance - used in hot paths
 */
inline AP_HAL::UARTDriver* AP_HAL::HAL::serial(uint8_t sernum) const
{
    if (sernum >= ARRAY_SIZE(serial_array)) {
        return nullptr;
    }
    return serial_array[sernum];
}
