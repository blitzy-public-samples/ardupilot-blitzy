/**
 * @file SITL_State.h
 * @brief Software In The Loop (SITL) simulation state management and integration
 * 
 * @details This file defines the main SITL_State class which manages the simulation
 *          environment for ArduPilot's Software-In-The-Loop testing. SITL allows
 *          running ArduPilot firmware on a development machine with simulated sensors,
 *          physics, and vehicle dynamics.
 *          
 *          Key responsibilities:
 *          - Integration with external physics simulators (JSBSim, Gazebo, XPlane, FlightGear, RealFlight)
 *          - Simulation of sensor inputs (IMU, GPS, barometer, airspeed, rangefinder)
 *          - Multicast output to visualization tools (FlightGear)
 *          - Command-line argument parsing for simulation configuration
 *          - Simulation rate control and time advancement
 *          - UDP networking for external simulator communication
 *          - HOME location and GPS origin configuration
 *          
 *          The SITL system operates by:
 *          1. Receiving physics updates from external simulator (FDM - Flight Dynamics Model)
 *          2. Converting physics data to simulated sensor readings
 *          3. Processing ArduPilot control outputs
 *          4. Sending servo/motor commands back to physics simulator
 *          5. Broadcasting state for visualization and analysis tools
 * 
 * @note SITL runs on Linux, Windows, macOS development platforms
 * @warning Real-time simulation constraints: CPU load affects simulation accuracy.
 *          High --speedup values may cause timing instability if CPU cannot keep up.
 * 
 * Source: libraries/AP_HAL_SITL/SITL_State.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SITL_State_common.h"

#if defined(HAL_BUILD_AP_PERIPH)
#include "SITL_Periph_State.h"
#else

class HAL_SITL;

/**
 * @class SITL_State
 * @brief Main state management class for Software-In-The-Loop simulation
 * 
 * @details SITL_State manages the complete simulation environment for ArduPilot firmware,
 *          bridging the gap between the flight controller code and external physics simulators.
 *          
 *          Architecture:
 *          - Inherits from SITL_State_Common which provides shared simulation functionality
 *          - Integrates with HAL components (Scheduler, Util, GPIO) as friend classes
 *          - Manages UDP socket connections to external simulators
 *          - Handles multicast broadcasting for visualization tools
 *          - Simulates sensor delays and noise characteristics
 *          
 *          Physics Integration:
 *          Supports multiple external Flight Dynamics Models (FDM):
 *          - JSBSim: Full-featured flight dynamics engine
 *          - Gazebo: 3D robot simulator with physics
 *          - X-Plane: Commercial flight simulator
 *          - FlightGear: Open-source flight simulator
 *          - RealFlight: RC flight simulator
 *          
 *          Simulation Flow:
 *          1. init() - Parse command-line args, setup networking, configure simulation
 *          2. loop_hook() - Called every scheduler iteration, steps simulation forward
 *          3. _fdm_input_step() - Receive physics updates from external simulator
 *          4. Update simulated sensors with physics data
 *          5. _simulator_servos() - Send control outputs back to simulator
 *          6. multicast_state_send() - Broadcast state for visualization
 *          
 *          Networking:
 *          - Base UDP port configurable via --base-port (default varies by vehicle)
 *          - FDM communication typically on base_port + 0
 *          - RC input on configurable --rcin-port
 *          - FlightGear visualization on --fg-view-port
 *          - Multicast output for state broadcasting
 *          
 *          Command-Line Configuration:
 *          - --home LAT,LON,ALT,HDG: Set initial GPS position and heading
 *          - --model MODEL: Select vehicle/physics model
 *          - --speedup FACTOR: Simulation speed multiplier (1.0 = realtime)
 *          - --rate RATE_HZ: Physics update rate in Hz
 *          - --base-port PORT: Base UDP port for networking
 *          - --serial0 PATH: UART port 0 configuration (GCS connection)
 *          
 * @note This class is only compiled when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * @warning Modifying simulation timing can affect stability of control algorithms.
 *          Always test with --speedup 1.0 before using higher values.
 * 
 * @see SITL_State_Common for base simulation functionality
 * @see libraries/SITL/ for vehicle-specific physics models
 * Source: libraries/AP_HAL_SITL/SITL_State.h
 */
class HALSITL::SITL_State : public SITL_State_Common {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    /**
     * @brief Initialize SITL simulation environment with command-line configuration
     * 
     * @details Performs complete SITL initialization sequence:
     *          - Parses command-line arguments (--home, --model, --speedup, --rate, etc.)
     *          - Configures HOME location and GPS origin
     *          - Sets up UDP networking for simulator communication
     *          - Initializes multicast sockets for visualization broadcasting
     *          - Configures serial port paths for UART simulation
     *          - Sets parameter defaults from command line
     *          - Establishes connection to external physics simulator (FDM)
     *          - Configures simulation timing and rate control
     *          
     *          Command-line options processed:
     *          --home LAT,LON,ALT,HDG - Initial position (can also be location name from locations.txt)
     *          --model MODEL          - Vehicle/physics model selection
     *          --speedup FACTOR       - Simulation speed (1.0=realtime, 10.0=10x speed)
     *          --rate RATE_HZ         - Physics update rate in Hz
     *          --base-port PORT       - Base UDP port for networking
     *          --rcin-port PORT       - RC input UDP port
     *          --fg-address IP        - FlightGear visualization IP address
     *          --serial0..8 PATH      - UART device configurations
     *          
     * @param[in] argc Argument count from main()
     * @param[in] argv Argument vector from main(), null-terminated strings
     * 
     * @note This overrides init() from SITL_State_Common base class
     * @note Called once during HAL initialization before scheduler starts
     * @warning Invalid HOME location will default to CMAC field (Canberra, Australia)
     * @warning Network port conflicts will cause simulation startup failure
     * 
     * @see _parse_command_line() for detailed argument parsing
     * @see parse_home() for HOME location string format
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void init(int argc, char * const argv[]) override;

    /**
     * @brief Main simulation update hook called every scheduler iteration
     * 
     * @details This is the core simulation update function called by the SITL scheduler
     *          at the configured simulation rate (typically 400Hz for copter, 50-400Hz for others).
     *          
     *          Execution sequence per loop:
     *          1. Check for incoming RC input via UDP
     *          2. Step the Flight Dynamics Model (_fdm_input_step)
     *          3. Update simulated sensor readings from FDM data
     *          4. Update rangefinder simulation
     *          5. Set height above ground level (AGL)
     *          6. Process control outputs (_simulator_servos)
     *          7. Send servo/motor commands to external simulator
     *          8. Broadcast state via multicast (multicast_state_send)
     *          9. Output visualization data to FlightGear (_output_to_flightgear)
     *          10. Advance simulation time
     *          
     * @note Called at main loop rate - must complete within loop time budget
     * @note Thread context: Runs in scheduler's main thread
     * @warning Blocking operations here will cause simulation timing issues
     * @warning CPU overload may cause simulation to fall behind wall-clock time
     * 
     * @see _fdm_input_step() for physics update details
     * @see _simulator_servos() for control output processing
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void loop_hook(void);
    
    /**
     * @brief Get the base UDP port for simulator networking
     * 
     * @details Returns the base port number used for SITL UDP communication.
     *          Various simulation services use offsets from this base port:
     *          - base_port + 0: FDM physics data exchange
     *          - base_port + 1: (reserved)
     *          - Other offsets for additional services
     *          
     *          Default base ports by vehicle type:
     *          - Copter: 5760
     *          - Plane: 5770
     *          - Rover: 5780
     *          - Sub: 5790
     *          
     * @return Base UDP port number (typically 5760-5799 range)
     * 
     * @note Port configurable via --base-port command-line argument
     * Source: libraries/AP_HAL_SITL/SITL_State.h:24-26
     */
    uint16_t base_port(void) const {
        return _base_port;
    }

    /**
     * @brief Check if RTS/CTS hardware flow control is enabled for serial ports
     * 
     * @details Returns whether hardware flow control (RTS/CTS) should be used
     *          for simulated UART devices. Affects serial port configuration
     *          when connecting to external tools.
     *          
     * @return true if RTS/CTS flow control enabled, false otherwise
     * 
     * @note Configurable via --rtscts command-line flag
     * @note Most SITL serial connections use TCP without flow control
     * Source: libraries/AP_HAL_SITL/SITL_State.h:28-30
     */
    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    /**
     * @brief Default configuration paths for UART device simulation
     * 
     * @details Defines the default connection strings for 9 simulated serial ports (SERIAL0-SERIAL8).
     *          Format: "tcp:PORT" or "tcp:PORT:wait" or device type like "GPS1"
     *          
     *          Default assignments:
     *          - SERIAL0 (console): "tcp:0:wait" - TCP port base+0, wait for connection (GCS link)
     *          - SERIAL1 (telem1):  "tcp:2"      - TCP port base+2
     *          - SERIAL2 (telem2):  "tcp:3"      - TCP port base+3
     *          - SERIAL3 (gps1):    "GPS1"       - First simulated GPS device
     *          - SERIAL4 (gps2):    "GPS2"       - Second simulated GPS device
     *          - SERIAL5-SERIAL8:   "tcp:5" through "tcp:8"
     *          
     *          Connection types:
     *          - "tcp:N" - TCP server on base_port + N
     *          - "tcp:N:wait" - TCP server that waits for connection before starting
     *          - "GPS1"/"GPS2" - Internal simulated GPS devices
     *          - "uart:PATH" - Connect to actual serial device
     *          
     * @note Overridden by --serial0 through --serial8 command-line arguments
     * @note GCS typically connects to SERIAL0 (tcp:0:wait)
     * Source: libraries/AP_HAL_SITL/SITL_State.h:33-43
     */
    const char *_serial_path[9] {
        "tcp:0:wait",
        "tcp:2",
        "tcp:3",
        "GPS1",
        "GPS2",
        "tcp:5",
        "tcp:6",
        "tcp:7",
        "tcp:8",
    };
    
    /**
     * @brief Storage for parameter defaults specified on command line
     * 
     * @details Holds parameter name/value pairs specified via command-line
     *          arguments (e.g., --defaults PARAM=VALUE). These override
     *          compiled-in parameter defaults before Parameters.cpp defaults
     *          are applied.
     *          
     *          Maximum 100 parameter overrides supported.
     *          
     * @note Parameters set this way take precedence over EEPROM values on first boot
     * Source: libraries/AP_HAL_SITL/SITL_State.h:44
     */
    ObjectArray<struct AP_Param::defaults_table_struct> cmdline_param{100};

    /**
     * @brief Parse HOME location string into Location and heading
     * 
     * @details Parses various HOME location formats:
     *          
     *          Numeric format: "LAT,LON,ALT,HDG"
     *          - LAT: Latitude in decimal degrees (-90 to +90)
     *          - LON: Longitude in decimal degrees (-180 to +180)
     *          - ALT: Altitude in meters above sea level
     *          - HDG: Initial heading in degrees (0-360, 0=North)
     *          
     *          Named location: "LOCATION_NAME"
     *          - Looks up location in Tools/autotest/locations.txt
     *          - Example: "CMAC" for Canberra Model Aircraft Club
     *          
     *          This sets the initial GPS position and vehicle heading for
     *          simulation startup. The GPS origin and EKF origin are both
     *          set to this location.
     *          
     * @param[in]  home_str      HOME location string to parse
     * @param[out] loc           Parsed geographic location (lat/lon/alt)
     * @param[out] yaw_degrees   Parsed initial heading in degrees
     * 
     * @return true if parsing successful, false if format invalid
     * 
     * @note Static method - can be called without SITL_State instance
     * @note Falls back to lookup_location() if numeric parse fails
     * @warning Invalid coordinates will cause parse to fail
     * 
     * @see lookup_location() for named location database
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    static bool parse_home(const char *home_str,
                           Location &loc,
                           float &yaw_degrees);

    /**
     * @brief Look up named location in locations database
     * 
     * @details Searches Tools/autotest/locations.txt for a named location
     *          and returns its coordinates and default heading. The locations
     *          file contains pre-defined test locations around the world.
     *          
     *          Format in locations.txt:
     *          LOCATION_NAME=LAT,LON,ALT,HDG
     *          
     *          Common locations:
     *          - CMAC: Canberra Model Aircraft Club (default)
     *          - KSFO: San Francisco Airport
     *          - YPDN: Bendigo, Australia
     *          - Plus many others for testing worldwide
     *          
     * @param[in]  home_str      Location name to look up
     * @param[out] loc           Location coordinates from database
     * @param[out] yaw_degrees   Default heading for this location
     * 
     * @return true if location found in database, false if not found
     * 
     * @note Static method - can be called without SITL_State instance
     * @note Case-sensitive location name matching
     * @warning Returns false if locations.txt cannot be opened
     * 
     * @see parse_home() which calls this for non-numeric strings
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    static bool lookup_location(const char *home_str,
                                Location &loc,
                                float &yaw_degrees);
    
    /**
     * @brief Get the SITL instance number for multi-instance simulation
     * 
     * @details Returns the instance ID for this SITL state, allowing multiple
     *          vehicle instances to run simultaneously with different configurations
     *          and network ports.
     *          
     * @return Instance number (0 for first instance, 1+ for additional instances)
     * 
     * @note Used to offset port numbers when running multiple vehicles
     * @note Instance configured via --instance command-line argument
     * Source: libraries/AP_HAL_SITL/SITL_State.h:56
     */
    uint8_t get_instance() const { return _instance; }

private:
    /**
     * @brief Parse command-line arguments for simulation configuration
     * 
     * @details Processes all SITL command-line options including:
     *          --home, --model, --speedup, --rate, --base-port, --rcin-port,
     *          --fg-address, --serial0..8, --defaults, --rtscts, --instance
     *          
     * @param[in] argc Argument count
     * @param[in] argv Argument vector
     * 
     * @note Calls _usage() and exits if invalid arguments detected
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _parse_command_line(int argc, char * const argv[]);
    
    /**
     * @brief Set a parameter default value from command line
     * 
     * @details Parses PARAM=VALUE string and adds to cmdline_param array
     * 
     * @param[in] parm Parameter string in format "PARAM_NAME=value"
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _set_param_default(const char *parm);
    
    /**
     * @brief Display command-line usage help and exit
     * 
     * @details Prints all available SITL command-line options with descriptions
     *          and exits the program. Called when invalid args detected.
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _usage(void);
    
    /**
     * @brief Perform SITL-specific initialization setup
     * 
     * @details Initializes SITL subsystems including simulated hardware,
     *          sensor emulation, and communication interfaces.
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _sitl_setup();
    
    /**
     * @brief Set up simulation timing and rate control
     * 
     * @details Configures simulation time advancement and loop rate control
     *          based on --speedup and --rate command-line options.
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _setup_timer(void);
    
    /**
     * @brief Set up ADC (Analog-to-Digital Converter) simulation
     * 
     * @details Initializes simulated ADC channels for analog sensor inputs
     *          like voltage/current monitoring and analog airspeed sensors.
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _setup_adc(void);

    /**
     * @brief Calculate and set current height above ground level (AGL)
     * 
     * @details Computes AGL altitude by comparing current position altitude
     *          with terrain elevation at current location. Used for rangefinder
     *          simulation and terrain-following features.
     * 
     * @note Called every simulation loop iteration
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void set_height_agl(void);
    
    /**
     * @brief Update simulated rangefinder sensor readings
     * 
     * @details Simulates rangefinder/lidar distance measurements based on:
     *          - Current height AGL
     *          - Vehicle attitude (rangefinder mounted angle)
     *          - Terrain slope and obstacles
     *          - Sensor noise and delay characteristics
     * 
     * @note Supports multiple rangefinder types (lidar, sonar, radar)
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _update_rangefinder();
    
    /**
     * @brief Set up Unix signal handlers for graceful shutdown
     * 
     * @details Installs signal handlers for SIGINT, SIGTERM to allow
     *          clean simulation shutdown and resource cleanup.
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _set_signal_handlers(void) const;

    /**
     * @brief Update simulated airspeed sensor with delay modeling
     * 
     * @details Applies realistic sensor delay and noise to airspeed measurements.
     *          Uses delay buffer to simulate sensor response time.
     * 
     * @param[in] airspeed True airspeed value from physics model in m/s
     * 
     * @note Delay configured by ARSPD_DELAY_MS parameter
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _update_airspeed(float airspeed);
    
    /**
     * @brief Read FDM input from local physics simulation
     * 
     * @details Receives Flight Dynamics Model data from built-in physics
     *          models running in same process. Alternative to external
     *          simulator integration.
     * 
     * @note Used with internal physics models (quadcopter, fixed-wing)
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _fdm_input_local(void);
    
    /**
     * @brief Send visualization data to FlightGear via UDP
     * 
     * @details Broadcasts current vehicle state to FlightGear flight simulator
     *          for 3D visualization. Sends position, attitude, velocity data
     *          formatted for FlightGear's multiplay protocol.
     *          
     * @note Enabled via --fg-view-port command-line option
     * @note IP address configurable via --fg-address (default: 127.0.0.1)
     * @warning UDP packet loss may cause visualization lag
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _output_to_flightgear(void);
    
    /**
     * @brief Process servo/motor outputs and send to physics simulator
     * 
     * @details Converts ArduPilot servo/motor output values (PWM) into
     *          format expected by external physics simulator. Handles:
     *          - PWM to normalized control input conversion
     *          - Output channel mapping
     *          - Mixing for combined controls
     *          - Transmission via UDP to FDM
     *          
     * @param[in] input Structure containing servo output values for all channels
     * 
     * @note Called every simulation loop after control calculations
     * @note Output format depends on selected physics simulator
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _simulator_servos(struct sitl_input &input);
    
    /**
     * @brief Step the Flight Dynamics Model forward one time step
     * 
     * @details Core simulation integration function that:
     *          1. Receives physics update from external simulator via UDP
     *          2. Extracts vehicle state (position, velocity, attitude, angular rates)
     *          3. Updates simulated sensor inputs (IMU, GPS, barometer, airspeed)
     *          4. Advances simulation time
     *          5. Maintains synchronization with physics simulator
     *          
     *          Supports multiple FDM types:
     *          - JSBSim: Detailed aircraft dynamics
     *          - Gazebo: General robotics simulation with ROS
     *          - X-Plane: High-fidelity flight simulation
     *          - FlightGear: Open-source flight simulator
     *          - RealFlight: RC aircraft simulator
     *          - Internal models: Built-in simple dynamics
     *          
     * @note Blocks waiting for physics data if running slower than wall-clock time
     * @warning Timeout if physics simulator stops responding
     * @warning UDP packet loss may cause sensor discontinuities
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void _fdm_input_step(void);

    /**
     * @brief Wait for simulation clock to advance to specified time
     * 
     * @details Implements simulation rate control by blocking until the
     *          simulation clock reaches the target time. Ensures loop rate
     *          consistency and --speedup factor accuracy.
     *          
     * @param[in] wait_time_usec Target simulation time in microseconds
     * 
     * @note Uses usleep() or similar platform sleep function
     * @warning CPU overload may prevent accurate timing
     * 
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void wait_clock(uint64_t wait_time_usec);

    // Internal state variables
    
    /**
     * @brief SITL instance number for multi-vehicle simulation
     * 
     * @details Instance ID used to differentiate multiple simultaneous vehicle
     *          simulations. Affects port number offsets and state identification.
     */
    uint8_t _instance;
    
    /**
     * @brief Base UDP port for simulator networking
     * 
     * @details Base port for all SITL UDP communication. Various services use
     *          offsets from this base (FDM on +0, RC input on configurable offset).
     */
    uint16_t _base_port;
    
    /**
     * @brief Process ID of parent process (for auto-shutdown detection)
     * 
     * @details Stores parent PID to detect if parent process terminates,
     *          allowing child SITL process to shutdown gracefully.
     */
    pid_t _parent_pid;
    
    /**
     * @brief Count of simulation update cycles since start
     * 
     * @details Incremented every loop_hook() call. Used for periodic tasks
     *          and performance monitoring.
     */
    uint32_t _update_count;

    /**
     * @brief Pointer to SITL scheduler instance
     * 
     * @details Reference to scheduler for time management and rate control.
     */
    Scheduler *_scheduler;

    /**
     * @brief UDP port for RC (radio control) input reception
     * 
     * @details Port number for receiving simulated RC input packets.
     *          Configurable via --rcin-port command-line option.
     */
    uint16_t _rcin_port;
    
    /**
     * @brief UDP port for FlightGear visualization output
     * 
     * @details Port for sending state data to FlightGear for 3D visualization.
     *          Configurable via --fg-view-port option.
     */
    uint16_t _fg_view_port;
    
    /**
     * @brief UDP port for IR-LOCK precision landing beacon simulation
     * 
     * @details Port for receiving IR-LOCK beacon detection data in simulation.
     */
    uint16_t _irlock_port;

    /**
     * @brief Enable RTS/CTS hardware flow control for serial ports
     * 
     * @details When true, simulated serial connections use flow control.
     */
    bool _use_rtscts;
    
    /**
     * @brief Enable FlightGear visualization output
     * 
     * @details When true, state data is sent to FlightGear on _fg_view_port.
     */
    bool _use_fg_view;
    
    /**
     * @brief IP address for FlightGear visualization destination
     * 
     * @details Target IP address for FlightGear visualization packets.
     *          Default: "127.0.0.1" (localhost).
     */
    const char *_fg_address;

    // Delay buffer configuration for sensor simulation
    
    /**
     * @brief Length of wind/airspeed sensor delay buffer
     * 
     * @details Number of samples stored for simulating sensor delay.
     *          50 samples allows up to 50ms delay at 1000Hz sampling.
     */
    static const uint8_t wind_buffer_length = 50;

    /**
     * @brief Structure for wind/airspeed delayed readings
     * 
     * @details Stores timestamped airspeed sensor measurement for delay modeling.
     */
    struct readings_wind {
        uint32_t time;  ///< Timestamp in microseconds
        float data;     ///< Airspeed reading in m/s
    };
    
    uint8_t store_index_wind;          ///< Current write position in circular buffer
    uint32_t last_store_time_wind;     ///< Timestamp of last buffer write (microseconds)
    VectorN<readings_wind,wind_buffer_length> buffer_wind;  ///< Circular delay buffer
    uint32_t time_delta_wind;          ///< Time step between samples (microseconds)
    uint32_t delayed_time_wind;        ///< Timestamp of delayed reading (microseconds)
    uint32_t wind_start_delay_micros;  ///< Configured sensor delay (microseconds)
    uint32_t last_wind_update_us;      ///< Last update timestamp (microseconds)

    /**
     * @brief Calculate simulated analog sonar sensor voltage
     * 
     * @details Converts simulated rangefinder distance to analog voltage (0-5V)
     *          as would appear on an ADC pin. Models analog rangefinder sensors
     *          like MaxBotix sonar.
     *          
     * @return Voltage between 0V and 5V representing distance
     * 
     * @note Voltage scales linearly with distance within sensor range
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    float _sonar_pin_voltage() const;

    // Multicast networking for external tool integration
    
    /**
     * @brief File descriptor for multicast state output socket
     * 
     * @details UDP socket for broadcasting vehicle state via multicast.
     *          Allows multiple tools to receive state data simultaneously.
     */
    int mc_out_fd = -1;
    
    /**
     * @brief File descriptor for multicast servo input socket
     * 
     * @details UDP socket for receiving servo override commands via multicast.
     *          Allows external tools to inject control inputs.
     */
    int servo_in_fd = -1;

    /**
     * @brief Open UDP multicast sockets for state broadcasting
     * 
     * @details Initializes multicast group membership and socket configuration
     *          for sending vehicle state to external monitoring/analysis tools.
     *          
     * @note Multicast address typically 239.255.145.50 (configurable)
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void multicast_state_open(void);
    
    /**
     * @brief Broadcast current vehicle state via UDP multicast
     * 
     * @details Sends complete vehicle state packet to multicast group including:
     *          - Position (lat/lon/alt)
     *          - Attitude (roll/pitch/yaw)
     *          - Velocities (NED frame)
     *          - Angular rates (body frame)
     *          - Servo/motor outputs
     *          
     * @note Called every simulation loop iteration
     * @note External tools can join multicast group to receive state
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void multicast_state_send(void);
    
    /**
     * @brief Update servo outputs from multicast input (external override)
     * 
     * @details Checks for and processes servo override commands received via
     *          multicast socket. Allows external tools to inject control inputs
     *          for testing or co-simulation.
     *          
     * @param[in,out] input Servo input structure to update with override values
     * 
     * @note Override commands have priority over ArduPilot outputs when active
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void multicast_servo_update(struct sitl_input &input);

    /**
     * @brief Array of servo output values for multicast broadcasting
     * 
     * @details Stores current servo/motor output values (PWM microseconds)
     *          for all channels to be sent via multicast. SITL_NUM_CHANNELS
     *          typically 16 channels.
     */
    uint16_t mc_servo[SITL_NUM_CHANNELS];
    
    /**
     * @brief Check for and process incoming servo override commands
     * 
     * @details Polls multicast servo input socket for override commands from
     *          external tools. Non-blocking check performed every loop.
     *          
     * @note Override commands allow HIL (Hardware-In-Loop) style testing
     * Source: libraries/AP_HAL_SITL/SITL_State.cpp
     */
    void check_servo_input(void);
};

#endif // defined(HAL_BUILD_AP_PERIPH)
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
