/**
 * @file AP_RCProtocol_UDP.h
 * @brief SITL UDP RC input protocol for simulation networking
 * 
 * @details This file implements RC (Remote Control) input reception via UDP
 *          datagrams for Software In The Loop (SITL) simulation environments.
 *          It provides a network-based RC input mechanism for testing and
 *          development without physical RC hardware.
 *          
 *          The UDP backend listens on port 5501 for RC input packets from
 *          simulators (e.g., FlightGear) or custom test tools. Packets contain
 *          PWM channel values as binary uint16_t arrays transmitted over UDP.
 *          
 *          This is a simulation-only component, compiled and active only in
 *          SITL builds for testing and development purposes.
 * 
 * @note This backend is only available when AP_RCPROTOCOL_UDP_ENABLED is defined
 * @warning SIMULATION ONLY - Not compiled or used in production flight builds
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_UDP_ENABLED

#include "AP_RCProtocol_Backend.h"

#include <AP_Common/missing/endian.h>
#include <AP_HAL/utility/Socket_native.h>

/**
 * @class AP_RCProtocol_UDP
 * @brief RC protocol backend for receiving RC input via UDP packets in SITL simulation
 * 
 * @details This backend implements network-based RC input reception for Software In The
 *          Loop (SITL) simulation environments. It creates a UDP socket listening on
 *          port 5501 to receive RC channel data from external simulators or test tools.
 *          
 *          **Architecture:**
 *          - Opens UDP socket on port 5501 (datagram mode, non-blocking)
 *          - Receives fixed-length packets containing 8 or 16 channels
 *          - Parses binary PWM values in network byte order (big-endian)
 *          - Feeds parsed channel values to autopilot RC input system
 *          - Maintains default PWM values (1500μs) for unset channels
 *          
 *          **Packet Format:**
 *          - Fixed-length UDP datagrams
 *          - Contains array of uint16_t PWM values in network byte order
 *          - Packet size determines channel count: 16 bytes = 8 channels, 32 bytes = 16 channels
 *          - Each channel value represents PWM pulse width in microseconds (1000-2000μs typical range)
 *          - Values are converted from network byte order (big-endian) to host byte order
 *          
 *          **Use Cases:**
 *          - SITL testing with external flight simulators (e.g., FlightGear, X-Plane)
 *          - Custom test tools sending RC commands via network
 *          - Automated testing frameworks injecting RC input
 *          - Multi-vehicle simulation with networked RC control
 *          - Hardware-in-the-loop (HIL) testing with virtual RC transmitters
 *          
 *          **Socket Configuration:**
 *          - Protocol: UDP (datagram)
 *          - Port: 5501 (hardcoded)
 *          - Mode: Non-blocking receive
 *          - Binding: Listens on all interfaces (0.0.0.0:5501)
 *          
 *          **Packet Validation:**
 *          - Channel count extracted from packet size (size / 2 = channels)
 *          - Maximum 16 channels supported
 *          - Invalid packet sizes are rejected
 *          - Endianness conversion applied to all channel values
 *          
 * @note This backend is automatically registered in SITL builds when UDP protocol is enabled
 * @warning SIMULATION ONLY - This backend is only compiled and active in SITL builds.
 *          It is not available in production flight controller firmware due to security
 *          and resource constraints. Never attempt to use this on actual hardware.
 * @warning Network security is not implemented - assumes trusted simulation environment.
 *          No authentication, encryption, or packet validation beyond basic format checks.
 * 
 * @see AP_RCProtocol_Backend for base class interface
 * @see AP_RCProtocol for protocol manager
 */
class AP_RCProtocol_UDP : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Update method to receive and process UDP RC input datagrams
     * 
     * @details Called periodically by the RC protocol manager to check for incoming
     *          UDP packets containing RC channel data. This method performs non-blocking
     *          socket reads to retrieve all available packets, parses the binary PWM
     *          values, converts from network byte order, and updates the RC input state.
     *          
     *          **Processing Flow:**
     *          1. Perform lazy socket initialization on first call (opens UDP port 5501)
     *          2. Read all available UDP packets from socket (non-blocking)
     *          3. Extract channel count from packet size (bytes / 2)
     *          4. Convert each uint16_t from network byte order to host byte order
     *          5. Update internal PWM channel array with received values
     *          6. Notify RC protocol manager of new data availability
     *          7. Update last input timestamp for timeout detection
     *          
     *          **Timing:**
     *          - Called at RC protocol update rate (typically 50-100Hz in SITL)
     *          - Non-blocking to avoid stalling main simulation loop
     *          - Processes multiple packets per call if available in socket buffer
     *          
     *          **Error Handling:**
     *          - Socket initialization failures are silently ignored (no RC input available)
     *          - Invalid packet sizes are discarded
     *          - Socket read errors (other than EAGAIN/EWOULDBLOCK) are logged
     *          - Maintains last known good RC values on packet loss
     * 
     * @return void
     * 
     * @note This method must be called regularly to maintain RC input responsiveness
     * @note Lazy initialization defers socket creation until first update() call
     * @warning Socket operations may fail in constrained environments - no RC failsafe triggered
     * 
     * @see read_all_socket_input() for actual packet reception implementation
     * @see set_default_pwm_input_values() for channel initialization
     */
    void update() override;

#if AP_RCPROTOCOL_FDM_ENABLED
    /**
     * @brief Set the FDM (Flight Dynamics Model) backend reference for RC fallback
     * 
     * @details Establishes a link to the FDM backend to enable fallback RC input
     *          when FDM provides RC data. This allows simulators that provide RC
     *          input through the FDM connection to be used as an alternative to
     *          separate UDP RC packets.
     * 
     * @param[in] _fdm_backend Pointer to the FDM backend instance to use for fallback
     * 
     * @return void
     * 
     * @note Only available when AP_RCPROTOCOL_FDM_ENABLED is defined
     * @see AP_RCProtocol_FDM for FDM backend implementation
     */
    void set_fdm_backend(class AP_RCProtocol_FDM *_fdm_backend) {
        fdm_backend = _fdm_backend;
    }
#endif

private:

    /**
     * @brief Initialize UDP socket for RC input reception
     * 
     * @details Performs one-time socket initialization: creates UDP socket,
     *          binds to port 5501, sets non-blocking mode, and configures for
     *          datagram reception. Called lazily on first update() to defer
     *          resource allocation until actually needed.
     * 
     * @return true if socket successfully initialized, false on failure
     * 
     * @note Initialization failures are non-fatal - RC input simply unavailable
     * @warning Only call once - subsequent calls have no effect (guarded by init_done flag)
     */
    bool init();
    
    /// @brief Initialization guard flag - true after successful init() call
    bool init_done;

    /// @brief Timestamp of last received RC packet in milliseconds (AP_HAL::millis() timebase)
    uint32_t last_input_ms;

    /**
     * @brief Read and process all available UDP packets from socket buffer
     * 
     * @details Non-blocking read loop that drains the socket receive buffer,
     *          processing each packet to extract PWM channel values. Continues
     *          reading until socket buffer is empty (EAGAIN/EWOULDBLOCK).
     *          Handles endianness conversion and channel count extraction.
     * 
     * @return void
     * 
     * @note Processes multiple packets per call to avoid buffer overflow
     * @see update() which calls this method
     */
    void read_all_socket_input(void);
    
    /**
     * @brief UDP socket for receiving RC input datagrams
     * 
     * @details Native socket implementation in datagram mode (UDP).
     *          Configured to listen on port 5501 in non-blocking mode.
     *          Constructor parameter "true" specifies datagram mode.
     */
    SocketAPM_native rc_in{true};  // "true" means "datagram"

    /**
     * @brief Internal PWM channel value array fed to autopilot RC input system
     * 
     * @details Stores up to 16 RC channel values as PWM pulse widths in microseconds.
     *          Packets received usually contain a subset of channels (8 or 16), which
     *          are inserted into this array. Unset channels maintain default value of
     *          1500μs (neutral position).
     *          
     *          Channel values represent standard RC PWM timing:
     *          - Typical range: 1000-2000μs
     *          - Neutral: 1500μs
     *          - Resolution: 1μs
     *          
     * @note Array size is fixed at 16 channels (maximum supported)
     */
    uint16_t pwm_input[16];
    
    /**
     * @brief Number of RC channels currently active/received
     * 
     * @details Tracks the count of channels extracted from the most recent packet.
     *          Determined by packet size: size_in_bytes / 2 = channel_count.
     *          Maximum value is 16 (array size limit).
     */
    uint8_t num_channels;

    /**
     * @brief Initialize PWM input array with default neutral values
     * 
     * @details Sets all 16 channels to 1500μs (neutral/center position).
     *          Called during initialization and when resetting RC state.
     *          Ensures safe default values for channels not provided in packets.
     * 
     * @return void
     */
    void set_default_pwm_input_values();

#if AP_RCPROTOCOL_FDM_ENABLED
    /**
     * @brief Pointer to FDM backend for RC input fallback
     * 
     * @details Reference to Flight Dynamics Model backend that may provide RC input
     *          through the simulation FDM connection as an alternative source.
     *          Only available when FDM protocol support is compiled in.
     */
    AP_RCProtocol_FDM *fdm_backend;
#endif
};


#endif  // AP_RCPROTOCOL_UDP_ENABLED
