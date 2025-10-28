/**
 * @file SITL_State_common.h
 * @brief Common state management for Software-In-The-Loop (SITL) simulation
 * 
 * @details This file defines the shared state and interface for SITL simulation,
 *          providing the foundation for all simulated hardware peripherals and
 *          sensor devices. SITL_State_Common manages the connection between
 *          ArduPilot's hardware abstraction layer (HAL) and the physics/sensor
 *          simulation models.
 *          
 *          The SITL system enables:
 *          - Testing vehicle code without physical hardware
 *          - Sensor simulation with configurable noise and failures
 *          - Peripheral device simulation (GPS, compass, rangefinders, etc.)
 *          - Network-based connection to physics simulators
 *          - Multi-vehicle and ride-along instance support
 *          
 *          Architecture:
 *          - SITL_State_Common: Base class with shared simulation state
 *          - Platform-specific derived classes provide OS integration
 *          - Physics simulation in libraries/SITL/SIM_*.cpp
 *          - HAL implementation in libraries/AP_HAL_SITL/
 *          
 * @note SITL is only available when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * @warning Thread-safety: Most members are accessed from main thread only.
 *          Multicast and serial I/O may involve background threads.
 * 
 * @see libraries/SITL/SITL.h for simulation parameters
 * @see libraries/AP_HAL_SITL/HAL_SITL_Class.h for HAL implementation
 * 
 * Source: libraries/AP_HAL_SITL/SITL_State_common.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

/** @brief Multicast IP address for broadcasting SITL state to external viewers */
#define SITL_MCAST_IP "239.255.145.51"

/** @brief Multicast UDP port for SITL state broadcast */
#define SITL_MCAST_PORT 20721

/** @brief UDP port for servo/motor output monitoring */
#define SITL_SERVO_PORT 20722

#include <AP_HAL/utility/Socket_native.h>
#include <SITL/SIM_SoloGimbal.h>
#include <SITL/SIM_ADSB.h>
#include <SITL/SIM_ADSB_Sagetech_MXS.h>
#include <SITL/SIM_EFI_Hirth.h>
#include <SITL/SIM_Vicon.h>
#include <SITL/SIM_VectorNav.h>
#include <SITL/SIM_MicroStrain.h>
#include <SITL/SIM_InertialLabs.h>
#include <SITL/SIM_AIS.h>
#include <SITL/SIM_GPS.h>

#include <SITL/SIM_SerialRangeFinder.h>

#include <SITL/SIM_Frsky_D.h>
#include <SITL/SIM_CRSF.h>
// #include <SITL/SIM_Frsky_SPort.h>
// #include <SITL/SIM_Frsky_SPortPassthrough.h>
#include <SITL/SIM_PS_LD06.h>
#include <SITL/SIM_PS_RPLidarA2.h>
#include <SITL/SIM_PS_RPLidarA1.h>
#include <SITL/SIM_PS_TeraRangerTower.h>
#include <SITL/SIM_PS_LightWare_SF45B.h>

#include <SITL/SIM_RichenPower.h>
#include <SITL/SIM_Loweheiser.h>
#include <SITL/SIM_FETtecOneWireESC.h>

#include <SITL/SIM_ELRS.h>

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <vector>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <SITL/SITL.h>
#include <SITL/SITL_Input.h>

class HAL_SITL;

/**
 * @class HALSITL::SITL_State_Common
 * @brief Base class managing shared state for Software-In-The-Loop simulation
 * 
 * @details SITL_State_Common provides the common infrastructure for simulating
 *          hardware peripherals and sensors in a software-only environment. This
 *          class maintains:
 *          
 *          **Simulated Peripheral Devices:**
 *          - GPS receivers (multiple instances with configurable behavior)
 *          - Rangefinders (serial and analog types)
 *          - ADSB transponders and receivers
 *          - RC radio systems (FrSky, CRSF, ELRS)
 *          - Gimbal controllers
 *          - External positioning systems (Vicon, VectorNav)
 *          - Inertial navigation systems (MicroStrain, InertialLabs)
 *          - EFI (Electronic Fuel Injection) systems
 *          - AIS (Automatic Identification System) receivers
 *          - Proximity sensors (lidar, ultrasonic)
 *          - ESC telemetry (FETtec OneWire)
 *          
 *          **Simulated Analog Sensors:**
 *          - Battery voltage and current monitors
 *          - Airspeed sensors (multiple instances)
 *          - Sonar/rangefinder analog inputs
 *          
 *          **RC and Motor Control:**
 *          - PWM input channels for RC receiver simulation
 *          - PWM output channels for motor/servo monitoring
 *          - Integration with physics simulation for control feedback
 *          
 *          **Communication Infrastructure:**
 *          - Serial device factory for UART path parsing
 *          - UDP multicast for state broadcasting
 *          - Socket-based connection to external simulators
 *          
 *          **Lifecycle:**
 *          1. Initialization: init() called with command-line arguments
 *          2. Device Creation: Peripherals instantiated based on configuration
 *          3. Update Loop: sim_update() called each scheduler tick
 *          4. Peripheral Updates: Each device updates sensor data
 *          5. State Broadcast: Multicast state to external viewers
 *          
 *          **Integration with Physics Simulation:**
 *          The sitl_model pointer connects to vehicle-specific physics in
 *          libraries/SITL/ (e.g., MultiCopter, Plane, Rover). Sensor data
 *          flows from physics model through SITL_State_Common to HAL drivers.
 *          
 *          **Sensor Noise and Failures:**
 *          Simulation parameters (in SITL class) control sensor noise,
 *          bias, drift, and failure injection for comprehensive testing.
 *          
 * @note Platform-specific subclasses (SITL_State in Scheduler.h) provide
 *       OS-dependent implementations for timing and I/O
 * 
 * @warning Thread Safety: Most members accessed from main simulation thread.
 *          Serial I/O and multicast may use background threads. Use appropriate
 *          locking when accessing shared state from multiple threads.
 * 
 * @warning Memory: Peripheral device pointers are dynamically allocated during
 *          initialization. Ensure proper cleanup in destructor if implemented.
 * 
 * @see SITL::Aircraft for physics simulation base class
 * @see SITL::SIM for simulation parameters and configuration
 * @see AP_HAL_SITL for HAL driver implementations
 * 
 * Source: libraries/AP_HAL_SITL/SITL_State_common.h:58-215
 */
class HALSITL::SITL_State_Common {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    /**
     * @brief Initialize SITL state with command-line arguments
     * 
     * @details Pure virtual function implemented by platform-specific subclasses.
     *          Parses command-line options, initializes simulated peripherals,
     *          establishes connections to physics simulators, and sets up I/O
     *          infrastructure.
     *          
     *          Common initialization tasks:
     *          - Parse --home, --model, --serial ports, --defaults
     *          - Load parameter defaults from specified file
     *          - Initialize network connections for simulators
     *          - Create simulated serial devices based on configuration
     *          - Set up multicast state broadcasting
     * 
     * @param[in] argc Argument count from main()
     * @param[in] argv Argument vector from main(), including executable name
     * 
     * @note Called once during HAL initialization before scheduler starts
     * @warning Must complete before any HAL drivers attempt device access
     * 
     * @see AP_HAL_SITL::Util::commandline_arguments for argument parsing
     */
    virtual void init(int argc, char * const argv[]) = 0;

    /**
     * @enum vehicle_type
     * @brief Vehicle type identifiers for SITL simulation
     * 
     * @details Used to select vehicle-specific physics models and default
     *          configurations. Physics simulation behavior differs significantly
     *          between vehicle types (e.g., multicopter vs fixed-wing dynamics).
     */
    enum vehicle_type {
        NONE,        ///< No vehicle type specified or initialization incomplete
        ArduCopter,  ///< Multicopter (quad, hex, octo, traditional heli, etc.)
        Rover,       ///< Ground vehicle (car, boat, submarine in surface mode)
        ArduPlane,   ///< Fixed-wing aircraft (including quadplanes)
        ArduSub,     ///< Underwater vehicle (ROV/AUV with depth control)
        Blimp        ///< Lighter-than-air vehicle (airship/blimp)
    };

    /**
     * @brief Factory method to create simulated serial device instances
     * 
     * @details Parses UART path specifications from --serial parameters and
     *          instantiates appropriate simulated device objects. Supports
     *          a wide variety of serial peripherals including:
     *          
     *          **Supported Device Types (name parameter):**
     *          - GPS: "gps", "gps2" - Simulated GPS receivers
     *          - Rangefinder: "lightware", "nmea", "maxbotix", "benewake"
     *          - ADSB: "adsb", "sagetech-mxs"
     *          - Gimbal: "gimbal"
     *          - External AHRS: "vectornav", "microstrain", "inertiallabs"
     *          - Positioning: "vicon"
     *          - Telemetry: "frsky-d", "crsf", "elrs"
     *          - Proximity: "sf45b", "terarangertower", "rplidar"
     *          - EFI: "megasquirt", "hirth"
     *          - ESC: "fetteconewire"
     *          - Marine: "ais"
     *          
     *          The arg parameter provides device-specific configuration
     *          (e.g., baud rate, device address, sensor orientation).
     * 
     * @param[in] name Device type string (e.g., "gps", "lightware", "adsb")
     * @param[in] arg Device-specific configuration string, may be nullptr
     * @param[in] portNumber UART port number (0-7) for this device
     * 
     * @return Pointer to created SerialDevice, or nullptr if name not recognized
     * 
     * @note Caller is responsible for managing device lifetime
     * @note Devices are typically created during init() based on --serial args
     * 
     * @see SITL::SerialDevice base class for device interface
     * @see AP_HAL_SITL::UARTDriver for serial port implementation
     * 
     * Source: Implementation in SITL_State.cpp
     */
    SITL::SerialDevice *create_serial_sim(const char *name, const char *arg, const uint8_t portNumber);

    /**
     * @brief Simulated analog sensor voltages for ADC peripheral emulation
     * 
     * @details These members simulate analog-to-digital converter (ADC) pins
     *          that would be present on physical hardware. Values are updated
     *          each simulation tick in sim_update() based on physics model
     *          state and configured sensor parameters.
     *          
     *          Voltage ranges: 0.0V to 5.0V (or 3.3V depending on ADC config)
     *          
     *          Used by analog sensor drivers (AP_RangeFinder_analog,
     *          AP_BattMonitor_Analog) to read simulated sensor values.
     */
    
    /** @brief Simulated sonar/rangefinder analog voltage (typically ADC pin 0) */
    float sonar_pin_voltage;
    
    /** @brief Simulated airspeed sensor voltages, one per sensor (typically ADC pin 1+) */
    float airspeed_pin_voltage[AIRSPEED_MAX_SENSORS];
    
    /** @brief Primary battery voltage monitor (typically ADC pin 13) */
    float voltage_pin_voltage;
    
    /** @brief Primary battery current monitor (typically ADC pin 12) */
    float current_pin_voltage;
    
    /** @brief Secondary battery voltage monitor (typically ADC pin 15) */
    float voltage2_pin_voltage;
    
    /** @brief Secondary battery current monitor (typically ADC pin 14) */
    float current2_pin_voltage;

    /**
     * @brief RC receiver input PWM values
     * 
     * @details Simulated RC receiver channel outputs in microseconds (μs).
     *          Updated by RC input simulation (SITL_Input) or external
     *          simulator connections. Values typically range 1000-2000μs.
     *          
     *          Used by AP_HAL_SITL::RCInput to provide RC data to vehicle code.
     * 
     * @note Array size SITL_RC_INPUT_CHANNELS (typically 16 channels)
     * @warning Values outside 900-2100μs may cause failsafes
     */
    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    
    /**
     * @brief Flag indicating new RC input data is available
     * 
     * @details Set to true when pwm_input[] is updated. RCInput driver
     *          clears this flag after reading values.
     */
    bool new_rc_input;
    
    /**
     * @brief Motor/servo output PWM values
     * 
     * @details Captures PWM outputs from vehicle motor mixing and servo
     *          control, typically 1000-2000μs for standard servos/ESCs.
     *          These values are sent to physics simulation to update
     *          motor thrust and control surface deflections.
     *          
     *          Updated by AP_HAL_SITL::RCOutput when vehicle writes servo outputs.
     * 
     * @note Array size SITL_NUM_CHANNELS (typically 16 outputs)
     */
    uint16_t pwm_output[SITL_NUM_CHANNELS];
    
    /**
     * @brief Flag indicating pwm_output[] contains valid data
     * 
     * @details Set to true after first RCOutput write. Physics simulation
     *          should wait for this before using pwm_output[] values.
     */
    bool output_ready = false;

#if AP_SIM_SOLOGIMBAL_ENABLED
    /**
     * @brief Enable flag for 3DR Solo gimbal simulation
     * 
     * @details When true, gimbal simulation is active and responds to
     *          MAVLink gimbal control commands.
     */
    bool enable_gimbal;
    
    /**
     * @brief Simulated 3DR Solo smart gimbal
     * 
     * @details Simulates MAVLink-controlled gimbal with attitude control.
     *          Responds to DO_MOUNT_CONTROL and MOUNT_CONFIGURE commands.
     * 
     * @note Only available when AP_SIM_SOLOGIMBAL_ENABLED
     */
    SITL::SoloGimbal *gimbal;
#endif

#if AP_SIM_ADSB_ENABLED
    /**
     * @brief Simulated ADS-B (Automatic Dependent Surveillance-Broadcast) transponder
     * 
     * @details Generates simulated aircraft traffic for collision avoidance testing.
     *          Creates ADSB_VEHICLE messages with configurable aircraft positions,
     *          velocities, and callsigns. Traffic patterns can simulate various
     *          encounter scenarios (head-on, overtaking, crossing, etc.).
     * 
     * @note Only available when AP_SIM_ADSB_ENABLED
     * @see SITL::ADSB for traffic generation parameters
     */
    SITL::ADSB *adsb;
#endif  // AP_SIM_ADSB_ENABLED

#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED
    /**
     * @brief Simulated Sagetech MXS ADS-B transponder
     * 
     * @details Emulates serial protocol for Sagetech MXS series transponders.
     *          Provides more realistic transponder behavior including configuration,
     *          mode selection, and status reporting.
     * 
     * @note Only available when AP_SIM_ADSB_SAGETECH_MXS_ENABLED
     */
    SITL::ADSB_Sagetech_MXS *sagetech_mxs;
#endif

#if !defined(HAL_BUILD_AP_PERIPH)
    /**
     * @brief Simulated Vicon motion capture system
     * 
     * @details Provides high-accuracy position and attitude data from simulated
     *          motion capture system. Used for testing indoor/GPS-denied flight
     *          with external positioning (VISION_POSITION_ESTIMATE messages).
     *          
     *          Simulates typical Vicon characteristics:
     *          - High update rate (100-200Hz)
     *          - Sub-centimeter position accuracy
     *          - Sub-degree attitude accuracy
     *          - Occasional tracking losses
     * 
     * @note Excluded from AP_Periph builds (periph devices don't use Vicon)
     */
    SITL::Vicon *vicon;
#endif

    /**
     * @brief Array of simulated serial rangefinder/lidar devices
     * 
     * @details Supports multiple serial rangefinder instances for multi-directional
     *          obstacle detection or redundant sensing. Each rangefinder can be
     *          configured with different:
     *          - Protocol (Lightware, Benewake, MaxBotix, NMEA, etc.)
     *          - Mounting orientation (forward, down, back, up, left, right)
     *          - Range limits and accuracy characteristics
     *          - Update rates
     *          
     *          Used for collision avoidance, terrain following, precision landing,
     *          and object detection testing.
     * 
     * @note Maximum 16 rangefinders supported
     * @see num_serial_rangefinders for count of active rangefinders
     */
    SITL::SerialRangeFinder *serial_rangefinders[16];
    
    /**
     * @brief Number of active serial rangefinders in the array
     * 
     * @details Valid rangefinder pointers are in serial_rangefinders[0] through
     *          serial_rangefinders[num_serial_rangefinders-1].
     */
    uint8_t num_serial_rangefinders;

    /**
     * @brief Simulated FrSky D-series telemetry radio
     * 
     * @details Emulates FrSky D-series telemetry protocol for testing downlink
     *          telemetry to FrSky transmitters. Simulates serial telemetry stream
     *          with vehicle data (battery, GPS, attitude, etc.).
     * 
     * @note Used for testing FrSky OSD and telemetry features
     */
    SITL::Frsky_D *frsky_d;
    // SITL::Frsky_SPort *frsky_sport;  // Future: S.Port telemetry
    // SITL::Frsky_SPortPassthrough *frsky_sportpassthrough;  // Future: Passthrough protocol

#if AP_SIM_PS_LD06_ENABLED
    /**
     * @brief Simulated LDROBOT LD06 360° lidar scanner
     * 
     * @details Emulates low-cost 2D lidar with 12m range and 4500Hz scan rate.
     *          Used for proximity sensing and simple SLAM applications.
     *          Generates realistic scan patterns with configurable noise.
     * 
     * @note Only available when AP_SIM_PS_LD06_ENABLED
     */
    SITL::PS_LD06 *ld06;
#endif  // AP_SIM_PS_LD06_ENABLED

#if AP_SIM_PS_RPLIDARA2_ENABLED
    /**
     * @brief Simulated Slamtec RPLidar A2 360° laser scanner
     * 
     * @details Emulates mid-range 2D lidar with 12m range and 10Hz scan rate.
     *          Provides 360° obstacle detection for collision avoidance.
     *          Supports both distance and signal quality simulation.
     * 
     * @note Only available when AP_SIM_PS_RPLIDARA2_ENABLED
     */
    SITL::PS_RPLidarA2 *rplidara2;
#endif

    /**
     * @brief Simulated FETtec OneWire ESC telemetry
     * 
     * @details Emulates FETtec OneWire ESC communication protocol providing:
     *          - ESC configuration and control
     *          - Telemetry (RPM, temperature, voltage, current)
     *          - Multi-ESC addressing on shared bus
     *          
     *          Used for testing ESC telemetry integration and motor monitoring.
     */
    SITL::FETtecOneWireESC *fetteconewireesc;

    /**
     * @brief Simulated Slamtec RPLidar A1 360° laser scanner
     * 
     * @details Emulates entry-level 2D lidar with 12m range and 5.5Hz scan rate.
     *          Lower-cost alternative to A2 for proximity sensing applications.
     */
    SITL::PS_RPLidarA1 *rplidara1;

#if AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
    /**
     * @brief Simulated LightWare SF45B 320° scanning lidar
     * 
     * @details Emulates advanced 50m range scanning lidar with:
     *          - 320° field of view (configurable scan sector)
     *          - Multiple distance returns per beam
     *          - Configurable resolution and update rates
     *          - Object tracking capabilities
     *          
     *          Used for advanced obstacle avoidance and terrain mapping.
     * 
     * @note Only available when AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
     */
    SITL::PS_LightWare_SF45B *sf45b;
#endif

#if AP_SIM_PS_TERARANGERTOWER_ENABLED
    /**
     * @brief Simulated TeraRanger Tower 360° proximity sensor array
     * 
     * @details Emulates 8-sensor ToF (Time of Flight) array providing
     *          360° obstacle detection with 4m range. Sensors positioned
     *          at 45° intervals for omnidirectional awareness.
     * 
     * @note Only available when AP_SIM_PS_TERARANGERTOWER_ENABLED
     */
    SITL::PS_TeraRangerTower *terarangertower;
#endif

#if AP_SIM_CRSF_ENABLED
    /**
     * @brief Simulated CRSF (Crossfire) RC and telemetry system
     * 
     * @details Emulates TBS Crossfire protocol providing:
     *          - High-resolution RC input (11-bit channels)
     *          - Bidirectional telemetry
     *          - Low-latency link simulation
     *          - Link quality metrics (RSSI, LQ)
     *          
     *          Used for testing CRSF radio integration and telemetry features.
     * 
     * @note Only available when AP_SIM_CRSF_ENABLED
     */
    SITL::CRSF *crsf;
#endif

    /**
     * @brief Simulated VectorNav INS/AHRS system
     * 
     * @details Emulates VectorNav VN-100/VN-200 inertial navigation systems
     *          providing high-accuracy:
     *          - Attitude estimation (quaternion output)
     *          - Inertial measurements (gyro, accel, mag)
     *          - GPS integration (VN-200)
     *          - Configurable output rates and data groups
     *          
     *          Used for testing external AHRS integration and redundant navigation.
     */
    SITL::VectorNav *vectornav;

    /**
     * @brief Simulated MicroStrain 3DM-GX5 series INS
     * 
     * @details Emulates MicroStrain GX5-series inertial navigation system
     *          with integrated GPS and GNSS-aided INS solution. Provides
     *          high-rate inertial data and position/velocity/attitude outputs.
     */
    SITL::MicroStrain5 *microstrain5;

    /**
     * @brief Simulated MicroStrain 3DM-GQ7 series INS
     * 
     * @details Emulates MicroStrain GQ7-series advanced INS with multi-GNSS
     *          and dual-antenna heading support. Newer generation than GX5.
     */
    SITL::MicroStrain7 *microstrain7;

    /**
     * @brief Simulated InertialLabs INS
     * 
     * @details Emulates InertialLabs AHRS/INS units providing:
     *          - Inertial and GNSS data fusion
     *          - High-accuracy position and attitude
     *          - Multiple output message formats
     *          
     *          Used for testing integration with InertialLabs products.
     */
    SITL::InertialLabs *inertiallabs;

#if AP_SIM_JSON_MASTER_ENABLED
    /**
     * @brief Ride-along vehicle instances via JSON SITL backend
     * 
     * @details Enables multiple vehicle instances to share a single physics
     *          simulation. The "master" vehicle runs full physics while
     *          "ride-along" instances receive sensor data via JSON protocol.
     *          
     *          Use cases:
     *          - Multi-vehicle formation flight testing
     *          - Cooperative missions with vehicle interaction
     *          - Leader-follower scenarios
     *          - Swarm behavior development
     *          
     *          Each ride-along instance runs independent flight code but
     *          shares the same simulated world state.
     * 
     * @note Only available when AP_SIM_JSON_MASTER_ENABLED
     * @see libraries/SITL/SIM_JSON.cpp for protocol details
     */
    SITL::JSON_Master ride_along;
#endif

#if AP_SIM_AIS_ENABLED
    /**
     * @brief Simulated AIS (Automatic Identification System) receiver
     * 
     * @details Emulates marine AIS receiver for vessel tracking and collision
     *          avoidance in maritime environments. Generates AIS messages with:
     *          - Vessel position, course, and speed
     *          - Vessel identification (MMSI, name, callsign)
     *          - Vessel dimensions and type
     *          
     *          Used for testing marine vehicle collision avoidance.
     * 
     * @note Only available when AP_SIM_AIS_ENABLED
     */
    SITL::AIS *ais;
    
    /**
     * @brief AIS message replay from recorded data
     * 
     * @details Replays previously captured AIS traffic for repeatable testing
     *          scenarios. Useful for regression testing with known traffic patterns.
     * 
     * @note Only available when AP_SIM_AIS_ENABLED
     */
    SITL::AIS_Replay *ais_replay;
#endif

    /**
     * @brief Simulated MegaSquirt EFI (Electronic Fuel Injection) system
     * 
     * @details Emulates MegaSquirt engine management system providing:
     *          - Engine RPM and temperature
     *          - Fuel consumption rate
     *          - Throttle position
     *          - Engine status and diagnostics
     *          
     *          Used for testing EFI integration on combustion engine vehicles.
     */
    SITL::EFI_MegaSquirt *efi_ms;

    /**
     * @brief Simulated Hirth EFI system
     * 
     * @details Emulates Hirth two-stroke aircraft engine EFI system with
     *          engine-specific telemetry and control protocol.
     */
    SITL::EFI_Hirth *efi_hirth;

    /**
     * @brief UDP socket for FlightGear visualization
     * 
     * @details Sends vehicle state to FlightGear flight simulator for 3D
     *          visualization. FlightGear provides realistic graphics and
     *          instrument panel displays for debugging and demonstrations.
     *          
     *          Connection typically on UDP port 5503 (configurable).
     * 
     * @note Socket is non-blocking to prevent simulation delays
     */
    SocketAPM_native fg_socket{true};
    
    /**
     * @brief Path to parameter defaults file
     * 
     * @details Points to file containing default parameter values loaded
     *          during initialization. Can be overridden with --defaults
     *          command-line argument.
     *          
     *          Default path defined by HAL_PARAM_DEFAULTS_PATH compile-time constant.
     */
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    /**
     * @brief Array of simulated GPS receiver instances
     * 
     * @details Supports multiple GPS receivers for redundancy and blending.
     *          Each GPS can be configured independently with:
     *          - GPS type (UBLOX, NMEA, SBF, GSOF, SIRF, MTK, etc.)
     *          - Satellite constellation (GPS, GLONASS, Galileo, BeiDou)
     *          - Accuracy characteristics and noise models
     *          - Position offsets (for RTK base/rover testing)
     *          - Jamming and interference simulation
     *          - Configurable dropouts and glitches
     *          
     *          Used for testing GPS failover, blending, and RTK positioning.
     * 
     * @note Array size AP_SIM_MAX_GPS_SENSORS (constrained by parameter sets)
     * @see SITL::GPS for per-instance configuration
     */
    SITL::GPS *gps[AP_SIM_MAX_GPS_SENSORS];

    /**
     * @brief Simulated ExpressLRS (ELRS) radio system
     * 
     * @details Emulates ExpressLRS long-range RC protocol with:
     *          - High update rates (50Hz to 500Hz)
     *          - Low latency
     *          - Telemetry downlink
     *          - Link quality and RSSI metrics
     *          
     *          Used for testing ELRS integration and high-rate RC input.
     */
    SITL::ELRS *elrs;

    /**
     * @brief Calculate simulated sonar/rangefinder analog voltage
     * 
     * @details Computes voltage that should appear on sonar ADC pin based on
     *          current simulated range to ground/obstacles. Implements sensor
     *          model converting distance to voltage using typical sonar
     *          characteristics (MaxBotix LV-EZ series model).
     *          
     *          Voltage calculation considers:
     *          - Distance to obstacle (from physics simulation)
     *          - Sensor range limits
     *          - Voltage scaling (typically ~9.8mV per cm)
     *          - Sensor noise and dropouts
     * 
     * @return Simulated voltage 0.0V to 5.0V
     * 
     * @note Called during sim_update() to update sonar_pin_voltage
     * @see sonar_pin_voltage member variable
     */
    float _sonar_pin_voltage() const;

    /**
     * @brief File descriptor for multicast output socket
     * 
     * @details Socket used to broadcast SITL state via UDP multicast.
     *          Value of -1 indicates socket is not open.
     */
    int mc_out_fd = -1;
    
    /**
     * @brief Initialize UDP multicast for SITL state broadcasting
     * 
     * @details Opens multicast socket bound to SITL_MCAST_IP:SITL_MCAST_PORT
     *          for broadcasting vehicle state to external monitoring tools
     *          and visualization applications.
     *          
     *          Multicast allows multiple receivers to monitor SITL state
     *          without individual connections.
     * 
     * @note Called during initialization if multicast enabled
     * @see multicast_state_send() for actual state transmission
     */
    void multicast_state_open(void);
    
    /**
     * @brief Broadcast current SITL state via UDP multicast
     * 
     * @details Sends vehicle position, attitude, velocity, and sensor states
     *          to multicast group. External tools can receive this data for:
     *          - Real-time visualization
     *          - Data logging and analysis
     *          - Multi-instance coordination
     *          
     *          Called periodically from main simulation loop.
     * 
     * @note Non-blocking send; dropped packets acceptable for monitoring
     * @see multicast_state_open() for initialization
     */
    void multicast_state_send(void);

    /**
     * @brief Counter for TCP serial queue overflow events
     * 
     * @details Tracks number of times simulation was paused (1ms sleep) because
     *          TCP transmit queue for serial port 0 was full. High counts indicate:
     *          - Ground station not consuming data fast enough
     *          - Network congestion or high latency
     *          - Excessive telemetry rate configuration
     *          
     *          Used for diagnosing SITL performance issues and tuning telemetry rates.
     * 
     * @warning Frequent overflows can slow simulation and affect timing accuracy
     */
    uint32_t _serial_0_outqueue_full_count;

protected:
    /**
     * @brief Current vehicle type for this simulation instance
     * 
     * @details Determines which physics model is loaded and which default
     *          parameters are applied. Set during initialization based on
     *          vehicle-specific code (ArduCopter, ArduPlane, etc.).
     * 
     * @see vehicle_type enum for possible values
     */
    enum vehicle_type _vehicle;

    /**
     * @brief Main simulation update function called each scheduler tick
     * 
     * @details Performs per-tick updates for all simulated peripherals and sensors:
     *          
     *          **Update Sequence:**
     *          1. Update physics model (sitl_model->update())
     *          2. Update analog sensor voltages (airspeed, sonar, battery)
     *          3. Update serial rangefinders with new distance readings
     *          4. Update RC radio input (FrSky, CRSF, ELRS telemetry)
     *          5. Update GPS positions and satellite data
     *          6. Update ADSB traffic positions
     *          7. Update gimbal state
     *          8. Update external AHRS systems (VectorNav, MicroStrain, etc.)
     *          9. Update proximity sensors (lidar scans)
     *          10. Update EFI engine telemetry
     *          11. Update battery model based on current draw
     *          12. Inject configured sensor noise and failures
     *          13. Broadcast state via multicast (if enabled)
     *          
     *          All sensor simulations use current vehicle state from physics model
     *          to generate realistic sensor readings with configured accuracy
     *          characteristics.
     * 
     * @note Called at main loop rate (typically 400Hz for copter, varies by vehicle)
     * @warning Must complete quickly to avoid simulation timing issues
     * 
     * @see SITL::Aircraft::update() for physics simulation step
     * @see update_voltage_current() for battery modeling
     */
    void sim_update(void);

    /**
     * @brief Pointer to vehicle physics simulation model
     * 
     * @details Points to vehicle-specific physics implementation:
     *          - SITL::MultiCopter for ArduCopter (quad, hex, octo, coax, etc.)
     *          - SITL::Plane for ArduPlane (includes quadplane support)
     *          - SITL::Rover for ground vehicles and boats
     *          - SITL::Submarine for ArduSub
     *          - SITL::Ballon for Blimp
     *          
     *          Physics model handles:
     *          - Force and moment calculations
     *          - 6-DOF dynamics integration
     *          - Ground/water contact modeling
     *          - Aerodynamic/hydrodynamic effects
     *          - Motor/propeller thrust models
     *          
     *          Updated each simulation tick with current control outputs
     *          (pwm_output[]) to compute vehicle motion.
     * 
     * @note Created during initialization based on --model argument
     * @see SITL::Aircraft base class for common physics interface
     * @see libraries/SITL/SIM_*.cpp for vehicle-specific implementations
     */
    SITL::Aircraft *sitl_model;

    /**
     * @brief Pointer to SITL simulation parameters singleton
     * 
     * @details Provides access to all simulation configuration parameters:
     *          
     *          **Parameter Categories:**
     *          - Sensor noise (gyro, accel, mag, baro, GPS)
     *          - Sensor biases and scale factors
     *          - Sensor failure injection (dropouts, stuck values)
     *          - Wind and turbulence models
     *          - Servo/motor dynamics (slew rates, lag)
     *          - Terrain and obstacle configuration
     *          - Battery and power system parameters
     *          - RC radio simulation settings
     *          
     *          Parameters can be set via:
     *          - MAVLink parameter protocol
     *          - Command-line arguments (--defaults)
     *          - SITL parameter file
     *          
     *          Changes to parameters affect sensor simulation behavior in real-time.
     * 
     * @note Singleton accessed via SITL::singleton()
     * @see SITL::SIM class definition in libraries/SITL/SITL.h
     * @see libraries/SITL/SIM_Params.cpp for parameter definitions
     */
    SITL::SIM *_sitl;

    /**
     * @brief Update battery voltage and current simulation
     * 
     * @details Implements battery model calculating voltage sag under load
     *          and current consumption based on motor/servo outputs. Models:
     *          - Voltage drop with current draw (internal resistance)
     *          - Capacity depletion over time
     *          - Pack voltage vs cell count
     *          - Multiple battery packs (voltage/voltage2, current/current2)
     *          
     *          Results written to voltage_pin_voltage, current_pin_voltage,
     *          voltage2_pin_voltage, current2_pin_voltage for ADC simulation.
     * 
     * @param[in] input Simulation input containing physics state
     * @param[in] throttle Normalized throttle value (0.0 to 1.0) for load calculation
     * 
     * @note Called from sim_update() each tick
     * @see voltage_pin_voltage, current_pin_voltage for output
     */
    void update_voltage_current(struct sitl_input &input, float throttle);
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
