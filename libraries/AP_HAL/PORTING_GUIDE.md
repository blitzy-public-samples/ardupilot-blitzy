# ArduPilot Hardware Abstraction Layer (HAL) Porting Guide

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [HAL Architecture Overview](#hal-architecture-overview)
- [HAL Interface Requirements](#hal-interface-requirements)
- [Device Driver Model](#device-driver-model)
- [Step-by-Step Porting Process](#step-by-step-porting-process)
- [Platform-Specific Implementation](#platform-specific-implementation)
- [Hardware Definition (hwdef) System](#hardware-definition-hwdef-system)
- [Testing and Validation](#testing-and-validation)
- [Troubleshooting](#troubleshooting)
- [Reference Implementations](#reference-implementations)

---

## Overview

This guide provides comprehensive instructions for porting ArduPilot to new hardware platforms by implementing a custom Hardware Abstraction Layer (HAL). The HAL provides a consistent interface between the ArduPilot core code and platform-specific hardware implementations, enabling ArduPilot to run on diverse autopilot hardware.

**Purpose**: The HAL abstracts hardware differences (UART, SPI, I2C, GPIO, storage, scheduling, etc.) allowing the vehicle-specific code (ArduCopter, ArduPlane, Rover, etc.) to remain platform-independent.

**Target Audience**: 
- Hardware manufacturers porting ArduPilot to new flight controllers
- Embedded systems engineers integrating ArduPilot with custom hardware
- Platform maintainers adding support for new operating systems or MCUs

**Key Concepts**:
- **HAL**: Hardware Abstraction Layer - the interface and implementation pattern
- **Backend/Driver**: Platform-specific implementation of a HAL interface
- **Device**: Unified abstraction for I2C/SPI/WSPI peripherals
- **hwdef**: Hardware definition system (ChibiOS-specific) for board configuration

---

## Prerequisites

### Required Knowledge

- **C++ Programming**: Strong understanding of C++11/14 features, virtual functions, inheritance
- **Embedded Systems**: Experience with bare-metal or RTOS development
- **Hardware Interfaces**: Familiarity with UART, SPI, I2C, GPIO, ADC, PWM
- **Build Systems**: Understanding of waf build system (or willingness to learn)
- **Debugging**: Experience with JTAG/SWD debugging, serial console debugging

### Development Environment

- **Cross-compilation toolchain**: ARM GCC for most flight controllers
- **Debugging hardware**: JTAG/SWD probe (J-Link, ST-Link, Black Magic Probe)
- **Version control**: Git for source code management
- **Hardware**: Target board with accessible debug interfaces

### Recommended Reading

Before starting, review these ArduPilot components:
- `libraries/AP_HAL/`: Core HAL interfaces
- `libraries/AP_HAL_Empty/`: Minimal HAL template
- `libraries/AP_HAL_ChibiOS/`: Most common reference implementation
- `libraries/AP_HAL_Linux/`: POSIX-based HAL for Linux systems
- `libraries/AP_HAL_SITL/`: Software-in-the-loop HAL for simulation

---

## HAL Architecture Overview

### System Architecture

The ArduPilot HAL follows a layered architecture:

```
┌─────────────────────────────────────────────────────────────┐
│  Vehicle Code (ArduCopter, ArduPlane, Rover, ArduSub)       │
│  - Uses HAL through singleton accessors: AP::hal()          │
└─────────────────────────────────────────────────────────────┘
                            │
┌─────────────────────────────────────────────────────────────┐
│  HAL Interface (libraries/AP_HAL/)                          │
│  - Abstract base classes defining required interfaces       │
│  - Device abstraction (I2C/SPI/WSPI)                       │
│  - Platform-independent API contracts                       │
└─────────────────────────────────────────────────────────────┘
                            │
┌─────────────────────────────────────────────────────────────┐
│  HAL Implementation (libraries/AP_HAL_XXX/)                 │
│  - Platform-specific driver implementations                 │
│  - Hardware initialization and configuration                │
│  - RTOS or bare-metal integration                          │
└─────────────────────────────────────────────────────────────┘
                            │
┌─────────────────────────────────────────────────────────────┐
│  Hardware / Operating System                                │
│  - MCU peripherals (STM32, etc.)                           │
│  - RTOS (ChibiOS, FreeRTOS, Linux)                         │
│  - Low-level drivers and BSP                               │
└─────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Interface Segregation**: Each HAL component has a focused responsibility
2. **Dependency Injection**: HAL components are injected via the HAL constructor
3. **Runtime Polymorphism**: Virtual functions enable platform-specific behavior
4. **Singleton Access**: Global HAL instance accessible via `AP::hal()`
5. **Threading Model**: Scheduler manages periodic and I/O tasks

---

## HAL Interface Requirements

The primary HAL interface is defined in `libraries/AP_HAL/HAL.h`. Every HAL implementation must provide all required driver components.

### Core HAL Class Structure

**Source**: `libraries/AP_HAL/HAL.h:21-161`

```cpp
class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _serial0,      // console
        AP_HAL::UARTDriver* _serial1,      // telem1
        AP_HAL::UARTDriver* _serial2,      // telem2
        AP_HAL::UARTDriver* _serial3,      // 1st GPS
        AP_HAL::UARTDriver* _serial4,      // 2nd GPS
        AP_HAL::UARTDriver* _serial5,      // extra1
        AP_HAL::UARTDriver* _serial6,      // extra2
        AP_HAL::UARTDriver* _serial7,      // extra3
        AP_HAL::UARTDriver* _serial8,      // extra4
        AP_HAL::UARTDriver* _serial9,      // extra5
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
        AP_HAL::DSP*        _dsp,
        AP_HAL::CANIface*   _can_ifaces[]);

    virtual void run(int argc, char * const argv[], Callbacks* callbacks) const = 0;

    // Public members accessible to application code
    AP_HAL::I2CDeviceManager* i2c_mgr;
    AP_HAL::SPIDeviceManager* spi;
    AP_HAL::WSPIDeviceManager* wspi;
    AP_HAL::AnalogIn*   analogin;
    AP_HAL::Storage*    storage;
    AP_HAL::UARTDriver* console;
    AP_HAL::GPIO*       gpio;
    AP_HAL::RCInput*    rcin;
    AP_HAL::RCOutput*   rcout;
    AP_HAL::Scheduler*  scheduler;
    AP_HAL::Util*       util;
    AP_HAL::OpticalFlow *opticalflow;
    AP_HAL::Flash*      flash;
    AP_HAL::DSP*        dsp;
    AP_HAL::CANIface*   can[HAL_NUM_CAN_IFACES];
};
```

### Required Driver Implementations

Each HAL implementation must provide concrete implementations of these driver interfaces:

#### 1. UARTDriver (Serial Communications)
**Interface**: `libraries/AP_HAL/UARTDriver.h`

**Responsibilities**:
- Asynchronous serial I/O with buffering
- Baud rate configuration
- Flow control (RTS/CTS) if hardware supports
- Support for RS-232, RS-485, UART protocols
- Typically need 10 instances (SERIAL0-SERIAL9)

**Key Methods to Implement**:
```cpp
virtual void begin(uint32_t baud_rate) = 0;
virtual void end() = 0;
virtual void flush() = 0;
virtual bool is_initialized() = 0;
virtual uint32_t available() = 0;
virtual uint32_t tx_pending() = 0;
virtual ssize_t read(uint8_t *buffer, uint16_t count) = 0;
virtual ssize_t write(const uint8_t *buffer, uint16_t size) = 0;
```

**Platform Considerations**:
- Use DMA for efficient transfers on resource-constrained MCUs
- Implement circular buffers for TX/RX queues
- Handle UART error conditions (overrun, framing, parity)

#### 2. I2CDeviceManager & SPIDeviceManager (Bus Management)
**Interfaces**: `libraries/AP_HAL/I2CDevice.h`, `libraries/AP_HAL/SPIDevice.h`

**Responsibilities**:
- Device enumeration and instantiation
- Bus arbitration and locking (semaphores)
- Multi-device support on shared buses
- DMA-accelerated transfers

**Key Methods to Implement**:
```cpp
// I2CDeviceManager
virtual OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) = 0;

// SPIDeviceManager
virtual OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) = 0;
```

**See**: [Device Driver Model](#device-driver-model) section for detailed information.

#### 3. AnalogIn (ADC)
**Interface**: `libraries/AP_HAL/AnalogIn.h`

**Responsibilities**:
- Analog-to-Digital conversion for voltage/current sensing
- Multi-channel ADC support
- Voltage scaling and calibration

**Key Methods**:
```cpp
virtual float read_average() = 0;
virtual float voltage_average() = 0;
```

**Typical Channels**:
- Battery voltage monitoring
- Battery current sensing
- RSSI measurement
- Servo rail voltage monitoring

#### 4. Storage (Persistent Parameters)
**Interface**: `libraries/AP_HAL/Storage.h`

**Responsibilities**:
- Non-volatile storage for parameters and configuration
- Wear-leveling for flash-based storage
- Atomic write operations

**Key Methods**:
```cpp
virtual void init() = 0;
virtual void read_block(void *dst, uint16_t src, size_t n) = 0;
virtual void write_block(uint16_t dst, const void *src, size_t n) = 0;
```

**Implementation Options**:
- Internal MCU flash with emulated EEPROM
- External SPI/I2C EEPROM or FRAM
- Filesystem-backed storage on SD card or eMMC

#### 5. GPIO (General Purpose I/O)
**Interface**: `libraries/AP_HAL/GPIO.h`

**Responsibilities**:
- Digital I/O pin control
- Pin mode configuration (input, output, pullup, pulldown)
- External interrupt handling

**Key Methods**:
```cpp
virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
virtual uint8_t read(uint8_t pin) = 0;
virtual void write(uint8_t pin, uint8_t value) = 0;
virtual bool attach_interrupt(uint8_t pin, AP_HAL::Proc p, uint8_t mode) = 0;
```

#### 6. RCInput (RC Receiver Input)
**Interface**: `libraries/AP_HAL/RCInput.h`

**Responsibilities**:
- Decoding RC receiver protocols (PWM, PPM, SBUS, DSM, CRSF, etc.)
- Multi-channel input capture
- Signal quality and failsafe detection

**Key Methods**:
```cpp
virtual void init() = 0;
virtual bool new_input() = 0;
virtual uint8_t num_channels() = 0;
virtual uint16_t read(uint8_t ch) = 0;
```

**Protocol Support**:
- PWM: Individual pins per channel, 1-2ms pulse width
- PPM: Single pin, combined channels
- SBUS: Inverted serial, 16 channels, digital
- DSM/DSM2/DSMX: Spektrum protocols
- CRSF: Crossfire (TBS), high refresh rate
- SRXL2: Spektrum Remote Receiver

#### 7. RCOutput (Servo/Motor Output)
**Interface**: `libraries/AP_HAL/RCOutput.h`

**Responsibilities**:
- PWM generation for servos and ESCs
- Multi-channel output generation
- DShot, OneShot, standard PWM protocols
- Output grouping and frequency control

**Key Methods**:
```cpp
virtual void init() = 0;
virtual void enable_ch(uint8_t ch) = 0;
virtual void disable_ch(uint8_t ch) = 0;
virtual void write(uint8_t ch, uint16_t period_us) = 0;
virtual void set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
virtual void set_output_mode(uint16_t mask, enum output_mode mode) = 0;
```

**Protocol Support**:
- Standard PWM: 50Hz (servos), 400Hz (multirotor)
- OneShot125: 125µs to 250µs pulses
- OneShot42: ~42µs pulses
- DShot150/300/600/1200: Digital ESC protocol
- Bi-directional DShot: ESC telemetry

#### 8. Scheduler (Task Management)
**Interface**: `libraries/AP_HAL/Scheduler.h`

**Responsibilities**:
- Main loop task scheduling
- Periodic task execution
- Thread/timer management
- System timing and delays
- Critical section management

**Key Methods**:
```cpp
virtual void init() = 0;
virtual void delay(uint16_t ms) = 0;
virtual void delay_microseconds(uint16_t us) = 0;
virtual void register_timer_process(AP_HAL::MemberProc) = 0;
virtual void register_io_process(AP_HAL::MemberProc) = 0;
virtual uint64_t micros64() = 0;
virtual uint32_t millis() = 0;
```

**Threading Model**:
- **Main Thread**: Vehicle loop (attitude control, navigation)
- **Timer Thread**: High-frequency sensor reads, fast loops
- **IO Thread**: Storage, logging, communications
- **RCIN Thread**: RC input processing
- **RCOUT Thread**: Motor/servo output

#### 9. Util (Utilities)
**Interface**: `libraries/AP_HAL/Util.h`

**Responsibilities**:
- System information and identification
- Performance counters
- Tone generation (buzzer)
- Safety switch handling
- Hardware status

**Key Methods**:
```cpp
virtual bool get_system_id(char buf[40]) = 0;
virtual bool toneAlarm_init() = 0;
virtual void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) = 0;
virtual bool safety_switch_state() = 0;
virtual void set_imu_temp(float current) = 0;
```

#### 10. OpticalFlow (Optional)
**Interface**: `libraries/AP_HAL/OpticalFlow.h`

**Responsibilities**:
- Integration with optical flow sensors (PX4FLOW, etc.)
- Sensor data acquisition

**Note**: Can provide NULL if not supported on your platform.

#### 11. Flash (Optional)
**Interface**: `libraries/AP_HAL/Flash.h`

**Responsibilities**:
- External flash memory management
- Dataflash logging backend

**Note**: Can provide NULL if using SD card or other logging mechanism.

#### 12. DSP (Optional - Advanced Platforms)
**Interface**: `libraries/AP_HAL/DSP.h`

**Responsibilities**:
- Hardware-accelerated signal processing
- FFT operations for ESC RPM telemetry

**Note**: Can provide NULL if DSP hardware unavailable.

#### 13. CANIface (CAN Bus - Optional)
**Interface**: `libraries/AP_HAL/CANIface.h`

**Responsibilities**:
- CAN bus communication for DroneCAN/UAVCAN
- Multi-CAN interface support

**Note**: Set to NULL if CAN not supported.

---

## Device Driver Model

ArduPilot uses a unified device abstraction for I2C, SPI, and WSPI (wide SPI - e.g., Quad-SPI) peripherals. This abstraction simplifies sensor driver development and promotes code reuse.

### Device Abstraction Overview

**Source**: `libraries/AP_HAL/Device.h:33-407`

The `AP_HAL::Device` class provides a bus-agnostic interface for peripheral access:

```cpp
class AP_HAL::Device {
public:
    enum BusType {
        BUS_TYPE_UNKNOWN = 0,
        BUS_TYPE_I2C     = 1,
        BUS_TYPE_SPI     = 2,
        BUS_TYPE_UAVCAN  = 3,
        BUS_TYPE_SITL    = 4,
        BUS_TYPE_MSP     = 5,
        BUS_TYPE_SERIAL  = 6,
        BUS_TYPE_WSPI    = 7,
    };

    // Core transfer operations
    virtual bool set_speed(Speed speed) = 0;
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) = 0;

    // Convenience wrappers
    bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len);
    bool write_register(uint8_t reg, uint8_t val, bool checked=false);

    // Periodic callback registration
    virtual PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) = 0;
    
    // Bus semaphore for synchronization
    virtual AP_HAL::Semaphore *get_semaphore() = 0;
};
```

### Key Device Features

#### 1. Unified Transfer Interface

The `transfer()` method provides half-duplex communication suitable for both I2C and SPI:
- **I2C**: Write address/command, then read response
- **SPI**: Write command, read response (CS managed automatically)

**Source**: `libraries/AP_HAL/Device.h:119-126`

```cpp
/**
 * This does a single bus transaction which sends send_len bytes, 
 * then receives recv_len bytes back from the slave. Operation is
 * half-duplex independent of bus type.
 *
 * Return: true on a successful transfer, false on failure.
 */
virtual bool transfer(const uint8_t *send, uint32_t send_len,
                      uint8_t *recv, uint32_t recv_len) = 0;
```

#### 2. Register Access Helpers

**Source**: `libraries/AP_HAL/Device.cpp:183-192`

```cpp
bool AP_HAL::Device::read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    uint8_t read_reg = first_reg;
    first_reg |= _read_flag;
    bool result = transfer(&first_reg, 1, recv, recv_len);
    if (_register_rw_callback != nullptr && result) {
        _register_rw_callback(read_reg, recv, recv_len, false);
    }
    return result;
}
```

**Purpose**: Simplifies reading multiple consecutive registers from sensors (e.g., IMU gyro XYZ data).

#### 3. Checked Registers for Safety-Critical Sensors

**Source**: `libraries/AP_HAL/Device.cpp:21-36`

ArduPilot supports runtime verification of critical sensor register values to detect brownouts or corruption:

```cpp
/*
  Using checked registers allows a device to check that a set of critical
  register values don't change at runtime. This is useful on key
  sensors (such as IMUs) which may experience brownouts or other
  issues in flight.

  To use register checking call setup_checked_registers() once to
  allocate the space for the checked register values. Then set the
  checked flag on any write_register() calls that you want protected.

  Periodically (say at 50Hz) you should then call
  check_next_register(). If that returns false then the sensor has had
  a corrupted register value. Marking the sensor as unhealthy is
  appropriate. The bad value will be corrected.
 */
```

**Implementation Steps**:
1. Allocate checked register storage: `setup_checked_registers(num_regs)`
2. Mark critical writes: `write_register(reg, val, true /* checked */)`
3. Periodically verify: `check_next_register()` in sensor update loop

**Source**: `libraries/AP_HAL/Device.cpp:112-154`

#### 4. Periodic Callbacks

Sensors typically need periodic polling at specific rates (e.g., 1kHz for IMU). The device abstraction provides thread-safe periodic callbacks:

**Source**: `libraries/AP_HAL/Device.h:260-273`

```cpp
/*
 * Register a periodic callback for this bus. All callbacks on the
 * same bus are made from the same thread with lock already taken. In
 * other words, the callback is not executed on the main thread (or the
 * thread which registered the callback), but in a separate per-bus
 * thread.
 *
 * After registering the periodic callback, the other functions should not
 * be used anymore from other contexts. If it really needs to be done, the
 * lock must be taken.
 *
 * Return: A handle for this periodic callback. To cancel the callback
 * call #unregister_callback() or return false on the callback.
 */
virtual PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) = 0;
```

**Usage Pattern**:
```cpp
// In sensor driver initialization
_dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&IMUDriver::_poll_data, void));
```

### Implementing Device Managers

Your HAL must provide concrete implementations of device managers:

#### I2CDeviceManager Implementation

**Responsibilities**:
- Create `I2CDevice` instances for a given bus and address
- Manage multiple I2C buses
- Implement bus-level locking

**Example Structure**:
```cpp
class I2CDevice : public AP_HAL::I2CDevice {
public:
    I2CDevice(uint8_t bus, uint8_t address);
    
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool set_speed(enum Speed speed) override;
    AP_HAL::Semaphore *get_semaphore() override;
    PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) override;
    
private:
    uint8_t _bus;
    uint8_t _address;
    AP_HAL::Semaphore *_sem;
    // Platform-specific I2C handle
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) override {
        return OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address));
    }
};
```

#### SPIDeviceManager Implementation

**Responsibilities**:
- Create `SPIDevice` instances by name
- Manage CS (chip select) pin assignments
- Coordinate DMA usage across devices

**Example Structure**:
```cpp
class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(const char *name, uint8_t bus, uint8_t cs_pin, uint32_t freq_hz);
    
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override;
    bool set_speed(enum Speed speed) override;
    AP_HAL::Semaphore *get_semaphore() override;
    
private:
    const char *_name;
    uint8_t _bus;
    uint8_t _cs_pin;
    uint32_t _frequency;
    // Platform-specific SPI handle
};
```

**Device Naming Convention**:
Device names typically match sensor names: `"mpu6000"`, `"bmi088_gyro"`, `"icm42688"`, `"ms5611"`, etc.

### Bus Identification System

**Source**: `libraries/AP_HAL/Device.cpp:222-274`

Each device has a unique 32-bit bus ID encoding bus type, bus number, device address, and device type:

```cpp
/**
 * make a bus id given bus type, bus number, bus address and
 * device type This is for use by devices that do not use one of
 * the standard HAL Device types, such as UAVCAN devices
 */
uint32_t AP_HAL::Device::make_bus_id(enum BusType bus_type, uint8_t bus, 
                                      uint8_t address, uint8_t devtype) {
    union DeviceId d {};
    d.devid_s.bus_type = bus_type;  // 3 bits
    d.devid_s.bus = bus;            // 5 bits
    d.devid_s.address = address;    // 8 bits
    d.devid_s.devtype = devtype;    // 8 bits
    return d.devid;
}
```

**Purpose**: This ID enables parameter storage and device identification across reboots, even when probing order changes.

---

## Step-by-Step Porting Process

This section provides a practical workflow for porting ArduPilot to a new hardware platform.

### Phase 1: Planning and Setup

#### 1.1 Choose Your Base HAL

Select the most appropriate existing HAL as your starting point:

| Base HAL | Best For | Characteristics |
|----------|----------|-----------------|
| **AP_HAL_ChibiOS** | STM32-based flight controllers | ChibiOS RTOS, rich peripheral support, most boards use this |
| **AP_HAL_Linux** | Linux SBCs (Raspberry Pi, BBB) | POSIX API, userspace drivers, file-based interfaces |
| **AP_HAL_SITL** | Simulation and testing | No hardware, TCP-based I/O simulation |
| **AP_HAL_Empty** | Minimal template | Starting point for completely new platforms |

**Recommendation**: For new flight controllers with ARM Cortex-M MCUs, use **AP_HAL_ChibiOS** as your reference.

#### 1.2 Set Up Build System Integration

Create a new board configuration in the waf build system:

**File**: `Tools/ardupilotwaf/boards.py`

Add a new board class:
```python
class my_new_board(Board):
    def configure_env(self, cfg, env):
        super(my_new_board, self).configure_env(cfg, env)
        
        env.CXXFLAGS += [
            '-DHAL_BOARD_MY_NEW_BOARD',
        ]
        
        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_MY_NEW_BOARD',
            HAL_BOARD_LOG_DIRECTORY = '"/logs"',
        )
```

#### 1.3 Define Board Identifier

**File**: `libraries/AP_HAL/AP_HAL_Boards.h`

```cpp
#define HAL_BOARD_SITL       3
#define HAL_BOARD_LINUX      4
#define HAL_BOARD_EMPTY      99
#define HAL_BOARD_CHIBIOS    10
#define HAL_BOARD_MY_NEW_BOARD  20  // Add your board ID
```

### Phase 2: Minimal HAL Implementation

Start with a minimal HAL that compiles and links, even if non-functional.

#### 2.1 Create HAL Directory Structure

```
libraries/AP_HAL_MyPlatform/
├── AP_HAL_MyPlatform.h           # Main header
├── AP_HAL_MyPlatform_Namespace.h # Namespace declarations
├── AP_HAL_MyPlatform_Private.h   # Internal declarations
├── HAL_MyPlatform_Class.h        # HAL implementation class
├── HAL_MyPlatform_Class.cpp      # HAL implementation
├── UARTDriver.h                  # UART driver interface
├── UARTDriver.cpp                # UART implementation
├── SPIDevice.h                   # SPI device interface
├── SPIDevice.cpp                 # SPI implementation
├── I2CDevice.h                   # I2C device interface
├── I2CDevice.cpp                 # I2C implementation
├── Scheduler.h                   # Scheduler interface
├── Scheduler.cpp                 # Scheduler implementation
├── Util.h                        # Utilities interface
├── Util.cpp                      # Utilities implementation
├── Storage.h                     # Storage interface
├── Storage.cpp                   # Storage implementation
├── GPIO.h                        # GPIO interface
├── GPIO.cpp                      # GPIO implementation
├── RCInput.h                     # RC input interface
├── RCInput.cpp                   # RC input implementation
├── RCOutput.h                    # RC output interface
├── RCOutput.cpp                  # RC output implementation
├── AnalogIn.h                    # Analog input interface
└── AnalogIn.cpp                  # Analog input implementation
```

#### 2.2 Implement HAL Main Class

**Template Source**: `libraries/AP_HAL_Empty/HAL_Empty_Class.h:1-12`

Create your main HAL class:

**File**: `libraries/AP_HAL_MyPlatform/HAL_MyPlatform_Class.h`

```cpp
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_MyPlatform_Namespace.h"

class HAL_MyPlatform : public AP_HAL::HAL {
public:
    HAL_MyPlatform();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

// Singleton accessor
const AP_HAL::HAL& AP_HAL::get_HAL();
```

**File**: `libraries/AP_HAL_MyPlatform/HAL_MyPlatform_Class.cpp`

**Template Source**: `libraries/AP_HAL_Empty/HAL_Empty_Class.cpp:1-80`

```cpp
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_MY_NEW_BOARD

#include "HAL_MyPlatform_Class.h"
#include "AP_HAL_MyPlatform_Private.h"

using namespace MyPlatform;

// Instantiate all driver objects
static UARTDriver serial0Driver;
static UARTDriver serial1Driver;
static UARTDriver serial2Driver;
static UARTDriver serial3Driver;
static I2CDeviceManager i2cDeviceManager;
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;
static Flash flashDriver;

HAL_MyPlatform::HAL_MyPlatform() :
    AP_HAL::HAL(
        &serial0Driver,  // console / SERIAL0
        &serial1Driver,  // telem1 / SERIAL1
        &serial2Driver,  // telem2 / SERIAL2
        &serial3Driver,  // GPS / SERIAL3
        nullptr,         // SERIAL4
        nullptr,         // SERIAL5
        nullptr,         // SERIAL6
        nullptr,         // SERIAL7
        nullptr,         // SERIAL8
        nullptr,         // SERIAL9
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,         // WSPI (if not supported)
        &analogIn,
        &storageDriver,
        &serial0Driver,  // console
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        nullptr          // DSP (if not supported)
    )
{}

void HAL_MyPlatform::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    /* 
     * Hardware initialization sequence:
     * 1. Initialize scheduler (sets up system clock, RTOS)
     * 2. Initialize peripherals (GPIO, UART, SPI, I2C)
     * 3. Initialize storage
     * 4. Call application setup
     * 5. Enter main loop
     */
    
    // Initialize scheduler first (configures system timing)
    scheduler->init();
    
    // Initialize console UART for early debugging
    serial(0)->begin(115200);
    
    // Initialize storage for parameter loading
    storage->init();
    
    // Initialize GPIO for board-specific setup
    gpio->init();
    
    // Call vehicle setup code
    callbacks->setup();
    
    // Mark system as initialized
    scheduler->set_system_initialized();
    
    // Enter main loop (never returns)
    for (;;) {
        callbacks->loop();
    }
}

// Singleton instance
static HAL_MyPlatform hal_myplatform;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_myplatform;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_myplatform;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_MY_NEW_BOARD
```

#### 2.3 Implement Namespace Headers

**File**: `libraries/AP_HAL_MyPlatform/AP_HAL_MyPlatform_Namespace.h`

```cpp
#pragma once

namespace MyPlatform {
    class UARTDriver;
    class I2CDevice;
    class I2CDeviceManager;
    class SPIDevice;
    class SPIDeviceManager;
    class AnalogIn;
    class Storage;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Util;
    class Semaphore;
    class OpticalFlow;
    class Flash;
}
```

### Phase 3: Core Driver Implementation

Implement drivers in order of dependency and importance for basic functionality.

#### 3.1 Scheduler (Highest Priority)

The scheduler is the foundation of the system - implement this first.

**Key Responsibilities**:
- System timing (`micros()`, `millis()`, `delay()`)
- Task registration and execution
- Thread management
- Semaphore/mutex primitives

**Implementation Checklist**:
- [ ] System timer (64-bit microsecond counter)
- [ ] Delay functions (busy-wait and yielding)
- [ ] Timer task registration (fast loop, typically 1kHz)
- [ ] IO task registration (slower tasks)
- [ ] Main loop integration
- [ ] Semaphore implementation for thread-safe access

**Example Timer Task Usage**:
```cpp
// In sensor driver
void IMU::init() {
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&IMU::_update, void));
}

void IMU::_update() {
    // Called at fast loop rate (typically 1kHz)
    _read_sensors();
    _update_filters();
}
```

#### 3.2 UART Driver

Critical for console output and debugging.

**Implementation Checklist**:
- [ ] Baud rate configuration
- [ ] TX/RX buffer management (ring buffers)
- [ ] Non-blocking I/O
- [ ] DMA support (recommended for efficiency)
- [ ] Multiple UART instance support
- [ ] Flow control (if hardware supports)

**Minimum Working Console**:
```cpp
void UARTDriver::begin(uint32_t baud_rate) {
    // Configure UART hardware
    // Enable TX/RX
    // Set baud rate
    _initialized = true;
}

ssize_t UARTDriver::write(const uint8_t *buffer, uint16_t size) {
    // Write to TX buffer or directly to hardware
    // Return number of bytes written
    return size;
}
```

**Testing**: Use `hal.console->printf("Hello ArduPilot!\n")` in `setup()` callback.

#### 3.3 GPIO

Required for peripheral control, LED indication, external interrupts.

**Implementation Checklist**:
- [ ] Pin mode configuration (input, output, pull-up, pull-down)
- [ ] Digital read/write
- [ ] External interrupt attachment
- [ ] Pin mapping (board-specific pin numbers to MCU pins)

#### 3.4 Storage

Required for parameter persistence.

**Implementation Checklist**:
- [ ] Non-volatile memory access (flash, EEPROM, FRAM)
- [ ] Wear leveling (for flash-based storage)
- [ ] Atomic write operations
- [ ] Storage size: Minimum 4KB, recommended 16KB

**Common Approaches**:
- **Internal Flash**: Use MCU flash with emulated EEPROM
- **External EEPROM/FRAM**: SPI or I2C attached memory
- **Filesystem**: Store parameters in a file on SD card

#### 3.5 I2C and SPI Device Managers

Critical for sensor interfacing.

**I2C Implementation Priority**:
- [ ] Basic I2C transfer (write-then-read pattern)
- [ ] Multi-master support (bus arbitration)
- [ ] Bus speed configuration (100kHz, 400kHz, 1MHz)
- [ ] Timeout and error handling
- [ ] Semaphore-based locking
- [ ] Periodic callback support

**SPI Implementation Priority**:
- [ ] SPI transfer (full-duplex and half-duplex)
- [ ] Chip select (CS) management
- [ ] Clock speed configuration
- [ ] Multiple SPI bus support
- [ ] DMA support (highly recommended)
- [ ] Semaphore-based locking
- [ ] Periodic callback support

**Testing**: Connect a common sensor (e.g., MS5611 barometer, MPU6000 IMU) and verify device ID read.

#### 3.6 AnalogIn

Required for battery voltage monitoring (safety-critical).

**Implementation Checklist**:
- [ ] Multi-channel ADC support
- [ ] Voltage scaling/calibration
- [ ] Averaging/filtering
- [ ] Typical channels: VBAT, IBAT, RSSI, 5V rail monitoring

#### 3.7 RCInput and RCOutput

Critical for vehicle control.

**RCInput Implementation**:
- [ ] At least one protocol (recommend starting with SBUS or PPM)
- [ ] Multi-channel decoding
- [ ] Failsafe detection
- [ ] Signal quality indication

**RCOutput Implementation**:
- [ ] Standard PWM output (50-490Hz)
- [ ] Multi-channel support (minimum 4, recommend 8+)
- [ ] Output grouping (channels sharing timers)
- [ ] OneShot125/OneShot42 (optional initially)
- [ ] DShot (optional, but increasingly expected)

### Phase 4: Testing and Validation

#### 4.1 Bring-Up Testing Sequence

**Test 1: Console Output**
```cpp
void setup() {
    hal.console->printf("ArduPilot HAL Test\n");
    hal.console->printf("Board: MY_NEW_BOARD\n");
}

void loop() {
    static uint32_t counter = 0;
    hal.console->printf("Loop %u, millis=%lu\n", counter++, hal.scheduler->millis());
    hal.scheduler->delay(1000);
}
```

**Expected Result**: Serial console output at 1Hz.

**Test 2: GPIO Toggle**
```cpp
void setup() {
    hal.gpio->pinMode(LED_PIN, HAL_GPIO_OUTPUT);
}

void loop() {
    hal.gpio->write(LED_PIN, 1);
    hal.scheduler->delay(500);
    hal.gpio->write(LED_PIN, 0);
    hal.scheduler->delay(500);
}
```

**Expected Result**: LED blinks at 1Hz.

**Test 3: Storage Read/Write**
```cpp
void test_storage() {
    uint8_t write_buf[16] = "ArduPilot";
    uint8_t read_buf[16] = {0};
    
    hal.storage->write_block(0, write_buf, sizeof(write_buf));
    hal.scheduler->delay(100);
    hal.storage->read_block(read_buf, 0, sizeof(read_buf));
    
    if (memcmp(write_buf, read_buf, sizeof(write_buf)) == 0) {
        hal.console->printf("Storage test PASSED\n");
    } else {
        hal.console->printf("Storage test FAILED\n");
    }
}
```

**Test 4: I2C Device Detection**
```cpp
void test_i2c() {
    // Try to probe for common barometer (MS5611) at 0x76
    auto dev = hal.i2c_mgr->get_device(0, 0x76);
    if (dev) {
        uint8_t whoami;
        if (dev->read_registers(0x00, &whoami, 1)) {
            hal.console->printf("I2C device found at 0x76\n");
        }
    }
}
```

**Test 5: SPI Device Access**
```cpp
void test_spi() {
    // Try to access IMU on SPI
    auto dev = hal.spi->get_device("mpu6000");
    if (dev) {
        uint8_t whoami;
        if (dev->read_registers(0x75, &whoami, 1)) {
            hal.console->printf("SPI IMU WHOAMI: 0x%02x\n", whoami);
        }
    }
}
```

---

## Platform-Specific Implementation

### ChibiOS-Based Implementation (STM32)

If targeting STM32-based flight controllers, leveraging ChibiOS provides significant advantages:

**Benefits of ChibiOS HAL**:
- Mature RTOS with preemptive multitasking
- Rich peripheral drivers (UART, SPI, I2C, ADC, PWM, CAN)
- DMA support throughout
- Low-level optimizations for Cortex-M
- Wide MCU support (STM32F4/F7/H7/G4/L4)

**Integration Approach**:
Use the existing `AP_HAL_ChibiOS` implementation and create a new hwdef (hardware definition) for your board rather than creating an entirely new HAL. See [Hardware Definition System](#hardware-definition-hwdef-system) section.

### Linux-Based Implementation

For single-board computers (Raspberry Pi, BeagleBone, etc.):

**Key Considerations**:
- Use POSIX APIs (pthreads, file I/O)
- Access hardware via `/dev` interfaces (`/dev/spidev*`, `/dev/i2c-*`)
- GPIO via sysfs or gpiod library
- RC input via file descriptors or shared memory
- RC output via PWM sysfs or kernel drivers

**Reference**: `libraries/AP_HAL_Linux/`

### Bare-Metal Implementation

For maximum control on resource-constrained platforms:

**Responsibilities**:
- Direct register access for all peripherals
- Interrupt handlers for UART RX, timers, DMA
- Manual memory management
- Custom scheduler (timer-based task dispatch)

**Challenges**:
- Higher development effort
- Platform-specific code
- Debugging complexity

**When to Choose**: Custom RTOS, non-standard MCU, extreme resource constraints.

---

## Hardware Definition (hwdef) System

For ChibiOS-based boards, ArduPilot uses a hardware definition system that generates board-specific code from declarative configuration files.

### Overview

**Location**: `libraries/AP_HAL_ChibiOS/hwdef/`

The hwdef system allows you to define your board's hardware configuration without writing low-level initialization code:
- Pin assignments (UART, SPI, I2C, GPIO)
- Device mappings (sensors, peripherals)
- Memory layout (flash, RAM)
- Bootloader configuration
- Default parameters

### Creating a New hwdef

#### Step 1: Create Board Directory

```bash
mkdir libraries/AP_HAL_ChibiOS/hwdef/MyNewBoard
cd libraries/AP_HAL_ChibiOS/hwdef/MyNewBoard
```

#### Step 2: Create hwdef.dat

**File**: `libraries/AP_HAL_ChibiOS/hwdef/MyNewBoard/hwdef.dat`

```
# Basic board definition
MCU STM32F4xx STM32F427xx
BOARD_TYPE 1420
define HAL_CHIBIOS_ARCH_FMUV3 1

# Clock configuration
OSCILLATOR_HZ 24000000
STM32_ST_USE_TIMER 5

# Flash and RAM
FLASH_SIZE_KB 2048
RAM_SIZE_KB 256

# Serial ports
SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 USART1
# USART2 (telem1)
PA2 USART2_TX USART2
PA3 USART2_RX USART2
# USART3 (telem2/GPS)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# SPI buses
# SPI1 - Internal sensors
PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI devices
PA4 MPU6000_CS CS
# Define SPI device
SPIDEV mpu6000 SPI1 DEVID1 MPU6000_CS MODE3 1*MHZ 4*MHZ

# I2C buses
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# ADC for battery monitoring
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)

# PWM output
# TIM1
PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
PE13 TIM1_CH3 TIM1 PWM(2) GPIO(51)
PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
PE9  TIM1_CH1 TIM1 PWM(4) GPIO(53)

# RC input
PA10 TIM1_CH3 TIM1 RCININT PULLDOWN LOW

# GPIOs
PB0 LED_ACTIVITY OUTPUT LOW
PB1 LED_BOOTLOADER OUTPUT LOW

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PC5  VBUS INPUT OPENDRAIN

# Bootloader embedding
ROMFS_WILDCARD libraries/AP_HAL_ChibiOS/hwdef/MyNewBoard/scripts/*.lua
```

#### Step 3: Create Bootloader Definition

**File**: `libraries/AP_HAL_ChibiOS/hwdef/MyNewBoard/hwdef-bl.dat`

```
# Bootloader-specific settings (minimal configuration)
include ../common/stm32f4_bl.dat

# Bootloader LED
PB0 LED_BOOTLOADER OUTPUT LOW

# Bootloader button (optional)
PB1 BTN_BOOTLOADER INPUT PULLUP
```

#### Step 4: Create defaults.parm

**File**: `libraries/AP_HAL_ChibiOS/hwdef/MyNewBoard/defaults.parm`

```
# Default parameters for this board
BATT_MONITOR 4
BATT_VOLT_PIN 0
BATT_CURR_PIN 1
BATT_VOLT_MULT 10.1
BATT_AMP_PERVLT 17.0

# IMU orientation (if needed)
AHRS_ORIENTATION 0

# Serial protocol defaults
SERIAL1_PROTOCOL 2   # MAVLink2
SERIAL2_PROTOCOL 5   # GPS
```

#### Step 5: Build the Firmware

```bash
./waf configure --board MyNewBoard
./waf copter
```

**Output**:
- `build/MyNewBoard/bin/arducopter.apj`: Firmware for upload via ground station
- `build/MyNewBoard/bin/arducopter_with_bl.hex`: Firmware with bootloader for initial flash

### hwdef Keywords Reference

| Keyword | Purpose | Example |
|---------|---------|---------|
| `MCU` | MCU family and specific model | `MCU STM32F4xx STM32F427xx` |
| `FLASH_SIZE_KB` | Flash memory size | `FLASH_SIZE_KB 2048` |
| `RAM_SIZE_KB` | RAM size | `RAM_SIZE_KB 256` |
| `SERIAL_ORDER` | Serial port enumeration order | `SERIAL_ORDER OTG1 USART2 USART3` |
| `SPIDEV` | Define SPI device | `SPIDEV mpu6000 SPI1 DEVID1 CS MODE3 1*MHZ 8*MHZ` |
| `IMU` | Define IMU sensor | `IMU Invensense SPI:mpu6000 ROTATION_YAW_180` |
| `BARO` | Define barometer | `BARO MS56XX I2C:0:0x76` |
| `COMPASS` | Define magnetometer | `COMPASS AK8963:probe_mpu9250 1 ROTATION_YAW_270` |
| `define` | C preprocessor define | `define HAL_STORAGE_SIZE 16384` |
| `DMA_PRIORITY` | Set DMA stream priority | `DMA_PRIORITY SPI1* ADC*` |

### Common hwdef Patterns

**Multi-IMU Configuration**:
```
SPIDEV mpu6000   SPI1 DEVID1 MPU6000_CS MODE3  1*MHZ  8*MHZ
SPIDEV icm20602  SPI4 DEVID2 ICM20602_CS MODE3 1*MHZ  8*MHZ

IMU Invensense  SPI:mpu6000 ROTATION_NONE
IMU Invensense  SPI:icm20602 ROTATION_NONE
```

**DShot Output**:
```
# DShot-capable outputs on TIM1
PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50) BIDIR
PE13 TIM1_CH3 TIM1 PWM(2) GPIO(51) BIDIR
PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52) BIDIR
PE9  TIM1_CH1 TIM1 PWM(4) GPIO(53) BIDIR

# Enable DShot
DMA_NOSHARE TIM1_UP TIM1_CH1 TIM1_CH2 TIM1_CH3 TIM1_CH4
DMA_PRIORITY TIM1_UP TIM1_CH1 TIM1_CH2 TIM1_CH3 TIM1_CH4
```

**External Barometer on I2C**:
```
BARO MS56XX I2C:0:0x76
```

### hwdef Scripts

The hwdef system uses Python scripts to process `.dat` files and generate C++ code:

**Location**: `libraries/AP_HAL_ChibiOS/hwdef/scripts/`

**Key Scripts**:
- `chibios_hwdef.py`: Main hwdef processor
- `STM32F4xx.py`, `STM32F7xx.py`, `STM32H7xx.py`: MCU-specific definitions
- `dma_resolver.py`: DMA conflict resolution

**Generated Files** (in build directory):
- `hwdef.h`: C++ header with pin definitions
- `hwdef.c`: Initialization code
- Board-specific HAL drivers

---

## Testing and Validation

### Unit Testing

Create unit tests for your HAL drivers using the ArduPilot test framework:

**Location**: `libraries/AP_HAL_MyPlatform/tests/`

**Example**: `test_uart.cpp`

```cpp
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(UARTTest, BasicIO) {
    hal.serial(0)->begin(115200);
    
    const char *test_string = "Hello";
    size_t written = hal.serial(0)->write((const uint8_t*)test_string, strlen(test_string));
    
    EXPECT_EQ(written, strlen(test_string));
}

AP_GTEST_MAIN()
```

### Hardware-in-the-Loop (HITL) Testing

Test your HAL with real sensors before full vehicle integration:

**Test Scenarios**:
1. **IMU Read Test**: Verify gyro and accelerometer data is reasonable
2. **Barometer Test**: Verify altitude readings and temperature
3. **Magnetometer Test**: Verify compass readings
4. **GPS Test**: Verify position fix and satellite count
5. **RC Input Test**: Verify channel values from receiver
6. **Motor Output Test**: Verify PWM/DShot output generation

**Example Test Program**:

```cpp
void setup() {
    hal.console->printf("HAL Hardware Test\n");
    
    // Initialize IMU
    auto imu_dev = hal.spi->get_device("mpu6000");
    if (imu_dev) {
        hal.console->printf("IMU device found\n");
    }
}

void loop() {
    // Read IMU data
    // Read baro data
    // Read GPS data
    // Print diagnostics
    
    hal.scheduler->delay(100);
}
```

### Software-in-the-Loop (SITL) Cross-Validation

Compare your HAL behavior against SITL:

1. Run same vehicle code on SITL and your hardware
2. Compare sensor data flow and timing
3. Verify equivalent parameter behavior
4. Test failsafe triggers and responses

### Compliance Testing

**Required Tests Before Production**:
- [ ] All sensor drivers functional (IMU, baro, mag, GPS)
- [ ] RC input working with common protocols
- [ ] Motor output working with standard PWM and DShot
- [ ] Parameter storage and retrieval working
- [ ] Logging functional (if supported)
- [ ] MAVLink communication working
- [ ] Arming and disarming sequence working
- [ ] Failsafe triggers responding correctly
- [ ] Flight modes switching correctly

### Performance Benchmarking

**Key Metrics**:
- **Main Loop Rate**: Should maintain target rate (typically 400Hz for copters)
- **IMU Sample Rate**: 1kHz minimum
- **Scheduler Jitter**: <1% variance
- **Memory Usage**: <80% of available RAM during flight
- **Flash Usage**: Room for future features

**Profiling Tools**:
```cpp
// Enable scheduler debug
hal.scheduler->set_debug_enabled(true);

// In main loop
static uint32_t loop_start = micros();
// ... loop code ...
uint32_t loop_time = micros() - loop_start;
if (loop_time > 2500) {  // 2.5ms for 400Hz loop
    hal.console->printf("WARNING: Loop overrun: %lu us\n", loop_time);
}
```

---

## Troubleshooting

### Common Issues and Solutions

#### Issue: Console Output Not Appearing

**Symptoms**: No serial console output, silent boot

**Troubleshooting**:
1. Verify UART TX pin configuration
2. Check baud rate (115200 is standard)
3. Confirm correct serial port on PC
4. Verify UART peripheral clock is enabled
5. Check TX buffer is being flushed
6. Test with simple GPIO toggle to confirm code is running

**Debug Approach**:
```cpp
// Add GPIO toggle to confirm execution
void hal_early_init() {
    // Configure LED pin directly via registers
    // Toggle LED in infinite loop
    while(1) {
        gpio_toggle(LED_PIN);
        for (volatile int i=0; i<100000; i++);
    }
}
```

#### Issue: Sensors Not Detected

**Symptoms**: Device probing fails, sensor init errors

**Troubleshooting**:
1. Verify SPI/I2C bus configuration (clock, MISO, MOSI, SCK pins)
2. Check chip select (CS) pin polarity and timing
3. Verify bus speed (try lowest speed first: 1MHz for SPI, 100kHz for I2C)
4. Confirm sensor power supply is stable
5. Use logic analyzer to capture bus transactions
6. Check pull-ups on I2C (4.7kΩ typical)

**Debug Code**:
```cpp
// SPI bus scan
void scan_spi_device(const char *name) {
    auto dev = hal.spi->get_device(name);
    if (!dev) {
        hal.console->printf("ERROR: Device %s not found\n", name);
        return;
    }
    
    // Try reading WHO_AM_I register (common at 0x75 or 0x00)
    uint8_t whoami;
    bool success = dev->read_registers(0x75, &whoami, 1);
    hal.console->printf("Device %s: whoami=0x%02x (success=%d)\n", 
                       name, whoami, success);
}

// I2C bus scan
void scan_i2c_bus(uint8_t bus) {
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        auto dev = hal.i2c_mgr->get_device(bus, addr);
        uint8_t dummy;
        if (dev && dev->read(&dummy, 1)) {
            hal.console->printf("I2C device found at 0x%02x\n", addr);
        }
    }
}
```

#### Issue: System Crashes or Hangs

**Symptoms**: Random resets, watchdog triggers, system freezes

**Troubleshooting**:
1. Check stack sizes (increase if needed)
2. Verify interrupt priorities don't cause deadlocks
3. Check for race conditions (missing semaphores)
4. Review DMA buffer alignment (some MCUs require specific alignment)
5. Check for buffer overflows
6. Enable stack overflow detection
7. Use watchdog for hang detection

**Debug Approach**:
```cpp
// Add watchdog to detect hangs
void watchdog_init() {
    // Configure hardware watchdog (platform-specific)
    // Timeout: 1-2 seconds
}

void loop() {
    watchdog_reset();  // Pet the dog
    
    // Main loop code
}
```

#### Issue: Timing Problems

**Symptoms**: Slow main loop, scheduler delays, jitter

**Troubleshooting**:
1. Verify system clock configuration (CPU frequency)
2. Check timer configuration (prescaler, period)
3. Profile slow functions (use GPIO toggles)
4. Ensure DMA is enabled for high-throughput peripherals
5. Check interrupt priorities (high-priority interrupts should be short)
6. Review compiler optimization flags

**Profiling**:
```cpp
#define TIMING_PIN 50  // GPIO pin for oscilloscope

void slow_function() {
    hal.gpio->write(TIMING_PIN, 1);
    
    // Function code
    
    hal.gpio->write(TIMING_PIN, 0);
}
// Measure pulse width with oscilloscope
```

#### Issue: Parameters Not Persisting

**Symptoms**: Parameters reset after reboot, storage writes fail

**Troubleshooting**:
1. Verify storage size (minimum 4KB)
2. Check flash write protection settings
3. Confirm erase operation completes before write
4. Verify storage initialization in boot sequence
5. Check for wear-leveling implementation (flash storage)

**Test Code**:
```cpp
void test_storage_persistence() {
    const uint8_t test_pattern[] = {0xAA, 0x55, 0xAA, 0x55};
    uint8_t read_back[4];
    
    hal.storage->write_block(0, test_pattern, 4);
    hal.scheduler->delay(100);  // Allow write to complete
    hal.storage->read_block(read_back, 0, 4);
    
    if (memcmp(test_pattern, read_back, 4) == 0) {
        hal.console->printf("Storage test PASSED\n");
    } else {
        hal.console->printf("Storage test FAILED\n");
        hal.console->printf("Wrote: %02x %02x %02x %02x\n", 
                           test_pattern[0], test_pattern[1], test_pattern[2], test_pattern[3]);
        hal.console->printf("Read:  %02x %02x %02x %02x\n", 
                           read_back[0], read_back[1], read_back[2], read_back[3]);
    }
}
```

#### Issue: RC Input Not Working

**Symptoms**: No stick input, channels read zero

**Troubleshooting**:
1. Verify protocol configuration (SBUS, PPM, etc.)
2. Check pin configuration (inverted UART for SBUS)
3. Verify signal levels with oscilloscope
4. Check baud rate (SBUS: 100000 baud, inverted)
5. Confirm receiver is bound and powered
6. Test with simple protocol first (PPM)

**Debug**:
```cpp
void test_rc_input() {
    hal.rcin->init();
    
    hal.scheduler->delay(1000);  // Wait for input
    
    hal.console->printf("RC Channels: ");
    for (uint8_t i = 0; i < hal.rcin->num_channels(); i++) {
        hal.console->printf("CH%d=%d ", i+1, hal.rcin->read(i));
    }
    hal.console->printf("\n");
}
```

### Debugging Tools and Techniques

#### JTAG/SWD Debugging

**Recommended Debuggers**:
- SEGGER J-Link (best for production)
- ST-Link V2/V3 (good for STM32)
- Black Magic Probe (open-source)

**GDB Configuration**:
```bash
# Connect to J-Link
arm-none-eabi-gdb build/MyBoard/bin/arducopter.elf
(gdb) target extended-remote :2331
(gdb) monitor reset
(gdb) load
(gdb) break main
(gdb) continue
```

#### Logic Analyzer

Essential for debugging SPI/I2C communication issues.

**Recommended Analyzers**:
- Saleae Logic (best software, expensive)
- DSLogic (good value)
- Any USB logic analyzer with sigrok support

**What to Capture**:
- SPI: CLK, MOSI, MISO, CS
- I2C: SCL, SDA
- UART: TX, RX
- RC Input: Signal pin
- PWM Output: Motor/servo pins

#### Serial Console Debugging

```cpp
// Add verbose debugging
#define DEBUG_HAL 1

#if DEBUG_HAL
  #define HAL_DEBUG(fmt, ...) hal.console->printf("HAL: " fmt "\n", ##__VA_ARGS__)
#else
  #define HAL_DEBUG(fmt, ...)
#endif

void some_function() {
    HAL_DEBUG("Entering some_function");
    // ...
    HAL_DEBUG("Variable x = %d", x);
}
```

### Getting Help

**Community Resources**:
- **ArduPilot Discourse**: https://discuss.ardupilot.org/
- **Discord**: ArduPilot Discord server
- **GitHub Issues**: https://github.com/ArduPilot/ardupilot/issues

**When Asking for Help, Provide**:
1. Hardware platform details (MCU, clock speed, peripherals)
2. Build configuration (board name, waf configure output)
3. Console log output
4. Logic analyzer captures (if relevant)
5. Code snippets showing your implementation
6. Steps to reproduce the issue

---

## Reference Implementations

### Study These HAL Implementations

#### AP_HAL_ChibiOS (Primary Reference)
**Location**: `libraries/AP_HAL_ChibiOS/`

**Best for**: STM32 Cortex-M based flight controllers

**Key Files to Study**:
- `Scheduler.cpp`: Advanced scheduler with thread pools
- `SPIDevice.cpp`: DMA-optimized SPI with device locking
- `RCOutput.cpp`: Complex PWM/DShot implementation
- `hwdef/common/`: Common board initialization

**Strengths**:
- Production-quality code
- Extensive hardware support
- Well-tested and optimized
- Rich feature set (DShot, DMA, multi-threading)

#### AP_HAL_Linux
**Location**: `libraries/AP_HAL_Linux/`

**Best for**: Linux single-board computers (RPi, BeagleBone)

**Key Files to Study**:
- `Scheduler.cpp`: POSIX thread-based scheduler
- `SPIDevice.cpp`: spidev file I/O
- `RCInput_RPI.cpp`: Raspberry Pi DMA-based PPM decode

**Strengths**:
- Userspace implementation (no kernel modules)
- POSIX API usage
- Filesystem-based peripheral access

#### AP_HAL_SITL
**Location**: `libraries/AP_HAL_SITL/`

**Best for**: Understanding HAL interface requirements

**Key Files to Study**:
- `Scheduler.cpp`: Simulated time and threading
- `SITL_State.cpp`: Main simulation interface

**Strengths**:
- Simplest HAL implementation
- No hardware dependencies
- Good for understanding HAL API contracts

#### AP_HAL_Empty (Template)
**Location**: `libraries/AP_HAL_Empty/`

**Best for**: Starting a new HAL from scratch

**Key Files**:
- `HAL_Empty_Class.cpp`: Minimal HAL structure
- All files are templates with minimal implementation

**Strengths**:
- Clean template
- No cruft or legacy code
- Easy to understand structure

### Code Examples from ChibiOS HAL

#### Example: SPI Device Implementation

**Source**: `libraries/AP_HAL_ChibiOS/SPIDevice.cpp` (simplified)

```cpp
bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    // Acquire bus semaphore
    if (!bus.sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // Assert chip select
    spiSelect(spi_dev);
    
    // Perform transfer
    if (send_len > 0) {
        spiSend(spi_dev, send_len, send);
    }
    if (recv_len > 0) {
        spiReceive(spi_dev, recv_len, recv);
    }
    
    // Deassert chip select
    spiUnselect(spi_dev);
    
    // Release semaphore
    bus.sem->give();
    
    return true;
}
```

#### Example: Periodic Callback Registration

**Source**: `libraries/AP_HAL_ChibiOS/Scheduler.cpp` (simplified)

```cpp
void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    // Add callback to timer thread task list
    _timer_procs[_num_timer_procs++] = proc;
}

// Timer thread (runs at 1kHz)
void Scheduler::_timer_thread(void)
{
    while (true) {
        // Execute all registered timer callbacks
        for (uint8_t i = 0; i < _num_timer_procs; i++) {
            _timer_procs[i]();
        }
        
        // Wait for next tick (1ms)
        chThdSleepMicroseconds(1000);
    }
}
```

### Documentation and Specifications

**ArduPilot Documentation**:
- Main docs: https://ardupilot.org/dev/
- HAL documentation: https://ardupilot.org/dev/docs/apmcopter-programming-basics.html

**Hardware References**:
- STM32 Reference Manuals: https://www.st.com/en/microcontrollers-microprocessors/
- ChibiOS Documentation: http://www.chibios.org/dokuwiki/
- Device Datasheets: Check manufacturer websites

**MAVLink Protocol**:
- Specification: https://mavlink.io/en/

---

## Summary

Porting ArduPilot to a new platform requires implementing the HAL interface defined in `libraries/AP_HAL/`. The process involves:

1. **Planning**: Choose appropriate base HAL and set up build integration
2. **Minimal HAL**: Create HAL structure and implement core drivers (Scheduler, UART, GPIO)
3. **Device Support**: Implement I2C/SPI device managers for sensor access
4. **Peripheral Drivers**: Complete RCInput, RCOutput, AnalogIn, Storage
5. **Testing**: Systematic testing from console output through full vehicle operation
6. **hwdef (ChibiOS)**: Use hardware definition system for declarative board configuration
7. **Validation**: Comprehensive testing against compliance checklist

**Key Success Factors**:
- Study existing HAL implementations (especially AP_HAL_ChibiOS)
- Implement and test incrementally (don't try to complete everything before testing)
- Use reference sensors with known-good drivers
- Leverage hardware debugging tools (JTAG/SWD, logic analyzer)
- Engage with ArduPilot community for guidance

**Estimated Effort**:
- **Basic HAL** (console + GPIO): 1-2 weeks
- **Core Drivers** (I2C/SPI/Scheduler): 2-4 weeks
- **Full HAL** (all drivers working): 2-3 months
- **Production Ready** (tested, optimized, documented): 4-6 months

This guide provides the foundation for creating a production-quality HAL implementation. Refer to the existing HAL implementations for detailed examples and best practices.

**Good luck with your ArduPilot porting project!**

---

*Document Version: 1.0*  
*Last Updated: 2024*  
*ArduPilot HAL Interface Version: 4.x+*
