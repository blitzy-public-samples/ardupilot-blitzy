/**
 * @file SITL_Periph_State.h
 * @brief SITL HAL state management for AP_Periph peripheral device simulation
 * 
 * @details This file provides the HAL state implementation for running AP_Periph
 *          firmware in Software-In-The-Loop (SITL) simulation mode. It enables
 *          testing of DroneCAN/UAVCAN peripheral devices (such as GPS, compass,
 *          rangefinder, airspeed sensors) without physical hardware.
 *          
 *          The peripheral SITL mode differs from normal vehicle SITL:
 *          - Runs AP_Periph firmware instead of vehicle firmware
 *          - Communicates with main vehicle SITL instance via UDP multicast
 *          - Simulates individual CAN peripheral nodes on the network
 *          - Uses dedicated base port (5760) for peripheral communication
 *          - Receives sensor data and vehicle state from main SITL instance
 *          - Sends servo/actuator commands back to main instance
 *          
 *          This enables comprehensive testing of:
 *          - DroneCAN peripheral firmware without hardware
 *          - Multi-node CAN network configurations
 *          - Peripheral failover and redundancy scenarios
 *          - Firmware updates and node configuration
 *          - Integration with vehicle firmware over simulated CAN bus
 * 
 * @warning This is ONLY compiled when both CONFIG_HAL_BOARD == HAL_BOARD_SITL
 *          AND HAL_BUILD_AP_PERIPH are defined. It is NOT used for normal
 *          vehicle SITL simulation.
 * 
 * @note Peripheral SITL instances typically connect to vehicle SITL via
 *       simulated DroneCAN interface, allowing realistic testing of the
 *       complete CAN network topology.
 * 
 * @see AP_Periph firmware in Tools/AP_Periph/
 * @see SITL_State_common.h for common SITL infrastructure
 * @see libraries/AP_DroneCAN/ for DroneCAN protocol implementation
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(HAL_BUILD_AP_PERIPH)

#include "SITL_State_common.h"

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>

#include <SITL/SITL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_HAL/utility/Socket_native.h>

/**
 * @class SimMCast
 * @brief Multicast communication handler for SITL peripheral simulation
 * 
 * @details SimMCast implements the network communication layer that connects
 *          a simulated peripheral device (running AP_Periph firmware in SITL)
 *          to the main vehicle SITL instance. It uses UDP multicast to:
 *          
 *          1. Receive vehicle state and sensor data from main SITL instance
 *          2. Send servo/actuator commands back to main SITL instance
 *          3. Synchronize timing between peripheral and vehicle simulations
 *          
 *          Communication Protocol:
 *          - Multicast group: Configured in SITL parameters
 *          - Base port: 5760 (default for peripheral communication)
 *          - Vehicle state updates: Position, attitude, velocities, timestamp
 *          - Servo commands: PWM outputs from peripheral firmware
 *          
 *          This architecture allows multiple peripheral SITL instances to
 *          coexist and communicate with a single vehicle SITL instance,
 *          simulating a realistic multi-node DroneCAN network topology.
 * 
 * @note Inherits from SITL::Aircraft to integrate with the SITL physics
 *       simulation framework, even though peripherals don't control physics.
 * 
 * @warning Timing synchronization is critical - peripheral must use vehicle's
 *          timestamp to maintain consistent simulation time across processes.
 * 
 * @see SITL::Aircraft base class for simulation interface
 * @see servo_send() for peripheral-to-vehicle servo output forwarding
 * @see multicast_read() for vehicle-to-peripheral state updates
 */
class SimMCast : public SITL::Aircraft {
public:
    /**
     * @brief Construct a SimMCast peripheral communication handler
     * 
     * @details Initializes the multicast communication system for a simulated
     *          peripheral device. Sets up network sockets for bidirectional
     *          communication with the main vehicle SITL instance.
     * 
     * @param[in] frame_str Frame type identifier (used for logging/identification)
     * 
     * @note Constructor does not open sockets - call multicast_open() and
     *       servo_fd_open() during initialization
     */
    SimMCast(const char *frame_str);
    
    /**
     * @brief Update peripheral simulation state from vehicle SITL
     * 
     * @details Called at each simulation timestep to:
     *          1. Read multicast vehicle state updates from main SITL instance
     *          2. Process servo/actuator outputs from peripheral firmware
     *          3. Send servo commands to main SITL instance via UDP
     *          4. Synchronize simulation timing with vehicle instance
     *          
     *          This maintains the peripheral's connection to the simulated
     *          vehicle environment and enables realistic sensor simulation
     *          based on vehicle state (position, velocity, attitude).
     * 
     * @param[in] input Servo/actuator outputs from AP_Periph firmware
     *                   (PWM values for any servos controlled by peripheral)
     * 
     * @note Override of SITL::Aircraft::update() - integrates peripheral
     *       into standard SITL update cycle
     * 
     * @see multicast_read() for vehicle state reception
     * @see servo_send() for servo command transmission
     */
    void update(const struct sitl_input &input) override;

private:
    /// UDP socket for receiving vehicle state multicast messages from main SITL instance
    SocketAPM_native sock{true};
    
    /// UDP socket for sending servo/actuator commands back to main SITL instance
    SocketAPM_native servo_sock{true};

    /**
     * @brief Time offset between vehicle multicast timestamp and local peripheral time
     * 
     * @details Used to synchronize the peripheral simulation clock with the main
     *          vehicle SITL instance. All peripheral timing must be aligned with
     *          vehicle timing to ensure coherent multi-process simulation.
     *          
     *          Calculated on first multicast reception and maintained throughout
     *          simulation to provide consistent time base.
     * 
     * @note Units: microseconds
     */
    uint64_t base_time_us;

    /**
     * @brief Initialize and bind multicast receive socket
     * 
     * @details Opens UDP socket and joins multicast group to receive vehicle
     *          state updates. Configures socket options for multicast reception
     *          and sets non-blocking mode for integration with SITL main loop.
     * 
     * @note Called during peripheral SITL initialization
     * @warning Failure to open multicast socket prevents peripheral from
     *          receiving vehicle state - peripheral sensors will not simulate correctly
     */
    void multicast_open();
    
    /**
     * @brief Read and process vehicle state from multicast socket
     * 
     * @details Receives UDP multicast packets containing vehicle state information:
     *          - Position (latitude, longitude, altitude)
     *          - Attitude (roll, pitch, yaw)
     *          - Velocities (NED frame)
     *          - Timestamp (for synchronization)
     *          
     *          Updates peripheral's view of vehicle state for realistic sensor
     *          simulation (e.g., GPS reports vehicle position, compass reports
     *          vehicle heading, rangefinder measures altitude).
     * 
     * @note Called from update() at each simulation timestep
     * @note Non-blocking - returns immediately if no data available
     * @see base_time_us for timestamp synchronization
     */
    void multicast_read();

    /**
     * @brief Transmit servo outputs to main SITL instance
     * 
     * @details Sends servo/actuator commands from peripheral firmware to the
     *          main vehicle SITL instance. This allows peripherals that control
     *          actuators (e.g., smart servos, ESCs with integrated controllers)
     *          to affect vehicle behavior in simulation.
     *          
     *          Servo data is packaged as UDP packet with:
     *          - Servo channel numbers
     *          - PWM values (typically 1000-2000 microseconds)
     *          - Timestamp
     * 
     * @note Called from update() after processing peripheral firmware outputs
     * @see servo_sock for transmission socket
     */
    void servo_send(void);
    
    /**
     * @brief Initialize and connect servo transmission socket
     * 
     * @details Opens UDP socket for sending servo commands to main SITL instance.
     *          Configures socket to send to vehicle's multicast group or unicast
     *          address depending on network configuration.
     * 
     * @note Called during peripheral SITL initialization
     * @warning Failure to open servo socket prevents peripheral from sending
     *          actuator commands to vehicle
     */
    void servo_fd_open(void);
};

class HAL_SITL;

/**
 * @class HALSITL::SITL_State
 * @brief Main SITL HAL state manager for AP_Periph peripheral firmware simulation
 * 
 * @details SITL_State manages the complete simulation environment for running
 *          AP_Periph firmware in Software-In-The-Loop mode. This enables testing
 *          of DroneCAN/UAVCAN peripheral devices without physical hardware.
 *          
 *          Key Responsibilities:
 *          - Initialize peripheral SITL environment (sockets, sensors, timing)
 *          - Manage UART connections for peripheral communication
 *          - Configure base port (default 5760) for peripheral networking
 *          - Simulate peripheral-specific sensors (GPS, compass, etc.)
 *          - Interface with main vehicle SITL via DroneCAN simulation
 *          - Provide HAL implementation for peripheral firmware
 *          
 *          Peripheral SITL Architecture:
 *          - AP_Periph firmware runs as separate process from vehicle SITL
 *          - Communicates with vehicle via simulated DroneCAN interface
 *          - Receives vehicle state via UDP multicast (SimMCast)
 *          - Simulates sensors based on vehicle position/attitude
 *          - Sends DroneCAN messages to vehicle firmware
 *          
 *          Common Use Cases:
 *          - Testing DroneCAN GPS peripheral firmware
 *          - Simulating redundant compass nodes
 *          - Testing rangefinder peripheral behavior
 *          - Validating airspeed sensor peripherals
 *          - Developing smart servo/ESC firmware
 *          - Testing multi-node CAN network configurations
 * 
 * @note Inherits from SITL_State_Common for shared SITL infrastructure
 *       but uses peripheral-specific configuration and communication.
 * 
 * @warning This class is ONLY compiled for AP_Periph builds. Do NOT use
 *          for normal vehicle SITL - use libraries/AP_HAL_SITL/SITL_State.h instead.
 * 
 * @see SimMCast for vehicle state multicast communication
 * @see AP_Periph firmware in Tools/AP_Periph/
 * @see libraries/AP_DroneCAN/ for simulated CAN protocol
 */
class HALSITL::SITL_State : public SITL_State_Common {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    /**
     * @brief Initialize peripheral SITL environment
     * 
     * @details Sets up complete simulation environment for AP_Periph firmware:
     *          1. Parse command-line arguments (instance number, ports, options)
     *          2. Configure base port (default 5760 + instance offset)
     *          3. Initialize UART paths for peripheral communication
     *          4. Set up simulated sensors (GPS, compass, etc.)
     *          5. Configure maintenance mode if requested
     *          6. Initialize timing and synchronization with vehicle SITL
     *          
     *          Command-line options include:
     *          - Instance number (for multi-peripheral simulation)
     *          - Base port override
     *          - UART device paths
     *          - Maintenance mode flag
     *          - Parameter defaults path
     * 
     * @param[in] argc Argument count from main()
     * @param[in] argv Argument vector from main()
     * 
     * @note Override of SITL_State_Common::init() with peripheral-specific setup
     * @warning Must be called before any other HAL operations
     * @see _base_port for default peripheral port configuration
     */
    void init(int argc, char * const argv[]) override;

    /**
     * @brief Check if RTS/CTS hardware flow control is enabled
     * 
     * @details Returns whether UART hardware flow control (RTS/CTS signaling)
     *          should be used for serial communication. Typically disabled for
     *          peripheral SITL as most peripherals use simple UART without
     *          flow control.
     * 
     * @return true if RTS/CTS flow control enabled, false otherwise
     * 
     * @note Configured via command-line argument during init()
     */
    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    /**
     * @brief Get the base UDP port for this peripheral instance
     * 
     * @details Returns the base network port used for peripheral communication.
     *          Default is 5760 for first peripheral, with subsequent instances
     *          offset by 10 (5770, 5780, etc.) to avoid port conflicts when
     *          running multiple peripheral SITL instances simultaneously.
     *          
     *          The base port is used for:
     *          - DroneCAN simulation communication
     *          - MAVLink serial-over-UDP (if configured)
     *          - Peripheral-to-vehicle state updates
     *          - GCS connection for peripheral monitoring
     * 
     * @return Base UDP port number for this peripheral instance
     * 
     * @note Default BASE_PORT is 5760, different from vehicle SITL (5760 offset)
     * @see _base_port configuration
     * @see _instance for multi-peripheral port calculation
     */
    uint16_t base_port(void) const {
        return _base_port;
    }

    /**
     * @brief UART device path configuration for peripheral SITL
     * 
     * @details Defines the connection paths for each serial port (UART) available
     *          to the peripheral firmware. Each entry specifies how that UART
     *          connects to external systems or simulated devices.
     *          
     *          Configuration format per entry:
     *          - "none:N" - UART disabled/not connected
     *          - "sim:device" - Connect to SITL simulated device
     *          - "udpclient:IP:PORT" - UDP client connection
     *          - "GPS1" - Connect to simulated GPS device
     *          
     *          Peripheral-specific UART usage:
     *          - UART 0-1: Typically disabled for peripherals
     *          - UART 2: Simulated ADSB receiver
     *          - UART 3: GPS device (for GPS peripheral simulation)
     *          - UART 4: UDP client for CAN-UART bridge testing
     *          - UART 5-8: Reserved/disabled
     * 
     * @note Array size [9] provides UART 0-8 (9 total serial ports)
     * @note Paths can be overridden via command-line arguments at startup
     * @warning Peripheral UART configuration differs from vehicle SITL
     *          - Peripherals typically use fewer UARTs
     *          - Different default connections for peripheral use cases
     * 
     * @see init() for command-line UART path override
     */
    const char *_serial_path[9] {
        "none:0",                        ///< UART 0: Disabled (no console for peripherals)
        "none:1",                        ///< UART 1: Disabled
        "sim:adsb",                      ///< UART 2: Simulated ADSB receiver
        "GPS1",                          ///< UART 3: Primary GPS device for GPS peripheral
        "udpclient:127.0.0.1:15550",     ///< UART 4: UDP client for CAN-UART testing
        "none:5",                        ///< UART 5: Disabled
        "none:6",                        ///< UART 6: Disabled
        "none:7",                        ///< UART 7: Disabled
        "none:8",                        ///< UART 8: Disabled
    };

    /**
     * @brief Get the instance number of this peripheral
     * 
     * @details Returns the instance number assigned to this peripheral SITL
     *          process. Instance numbers enable running multiple peripheral
     *          simulations simultaneously, each with unique network ports
     *          and identifiers.
     *          
     *          Instance number affects:
     *          - Base port calculation (5760 + instance*10)
     *          - DroneCAN node ID assignment
     *          - Parameter storage location
     *          - Log file naming
     * 
     * @return Instance number (0-based, typically 0-9)
     * 
     * @note Instance 0 is default if not specified on command line
     * @see base_port() for instance-based port calculation
     */
    uint8_t get_instance() const { return _instance; }

    /**
     * @brief Check if peripheral is running in maintenance mode
     * 
     * @details Maintenance mode is a special operating mode where peripheral
     *          firmware runs with reduced functionality for diagnostics,
     *          parameter configuration, or firmware updates without requiring
     *          full sensor initialization or vehicle connection.
     *          
     *          In maintenance mode:
     *          - Sensors may not be initialized
     *          - CAN communication may be limited
     *          - Parameter access is available
     *          - Firmware update is possible
     * 
     * @return true if running in maintenance mode, false for normal operation
     * 
     * @note Configured via --maintenance command-line flag
     * @warning Some peripheral functionality may be unavailable in maintenance mode
     */
    bool run_in_maintenance_mode() const { return _maintenance; }

private:

    /**
     * @brief Wait for simulated time to advance to specified timestamp
     * 
     * @details Synchronizes peripheral SITL execution with simulation time.
     *          Blocks until the simulation clock reaches the target time,
     *          maintaining timing consistency between peripheral and vehicle
     *          SITL instances.
     *          
     *          This is critical for:
     *          - Synchronized sensor updates
     *          - Consistent CAN message timing
     *          - Realistic peripheral behavior relative to vehicle
     * 
     * @param[in] wait_time_usec Target simulation time in microseconds
     * 
     * @note Uses SITL timing system, not wall-clock time
     * @warning Blocking call - do not use in time-critical interrupt context
     */
    void wait_clock(uint64_t wait_time_usec);
    
    /// RTS/CTS hardware flow control enable flag for UART connections
    bool _use_rtscts;
    
    /**
     * @brief Base UDP port for peripheral network communication
     * 
     * @details Default base port is 5760, offset from vehicle SITL base port
     *          to prevent conflicts. Multiple peripheral instances use
     *          _base_port + (_instance * 10) for unique port assignment.
     * 
     * @note Configurable via --base-port command-line argument
     */
    uint16_t _base_port;

    /**
     * @brief Path to parameter defaults file
     * 
     * @details Specifies location of default parameter values to load during
     *          peripheral initialization. Uses HAL_PARAM_DEFAULTS_PATH macro
     *          which is defined per board/platform.
     *          
     *          For SITL peripherals, this typically points to:
     *          Tools/AP_Periph/defaults.parm or board-specific defaults
     * 
     * @note Can be overridden via --defaults command-line argument
     */
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    /**
     * @brief Peripheral instance number for multi-peripheral simulation
     * 
     * @details Uniquely identifies this peripheral when multiple SITL
     *          peripheral instances run simultaneously. Affects port numbers,
     *          node IDs, and storage paths.
     * 
     * @note Range: 0-255, typically 0-9 in practice
     */
    uint8_t _instance;
    
    /**
     * @brief Maintenance mode operation flag
     * 
     * @details When true, peripheral operates in limited maintenance mode
     *          for diagnostics, configuration, or firmware updates without
     *          full operational capabilities.
     * 
     * @see run_in_maintenance_mode()
     */
    bool _maintenance;

    /**
     * @brief Array of simulated GPS device instances
     * 
     * @details Provides GPS sensor simulation for GPS peripheral firmware.
     *          Each GPS instance simulates a complete GPS receiver with:
     *          - Position (from vehicle SITL state)
     *          - Velocity and heading
     *          - Satellite visibility and signal quality
     *          - Fix type and accuracy metrics
     *          
     *          Array size [1] is limited by number of GPS parameter sets
     *          defined for peripheral firmware. Can be expanded if multiple
     *          GPS peripherals need simulation.
     * 
     * @note GPS data sourced from vehicle SITL via multicast communication
     * @see SITL::GPS class for simulated GPS implementation
     */
    SITL::GPS *gps[1];  // constrained by # of parameter sets
};

#endif
