/**
 * @file AP_RCProtocol_FDM.h
 * @brief SITL Flight Dynamics Model RC input bridge for simulation environment
 * 
 * @details This file implements the RC protocol backend for Software-In-The-Loop
 *          (SITL) simulation, enabling RC input to be sourced from the flight
 *          dynamics model rather than physical hardware. This bridge allows the
 *          simulator to inject control inputs directly into ArduPilot's RC
 *          processing pipeline for testing and development without hardware.
 * 
 *          The FDM backend receives normalized control inputs (-1.0 to +1.0) from
 *          the SITL::SIM class and converts them to standard PWM microsecond
 *          values (1000-2000μs) that match physical RC receiver output. This
 *          enables seamless testing of flight modes, control algorithms, and
 *          autonomous functions in simulation.
 * 
 *          Integration points:
 *          - SITL physics simulation (joystick/gamepad input)
 *          - MAVProxy RC override commands
 *          - Automated test scenarios via scripted inputs
 *          - Hardware-in-the-loop (HIL) simulation bridges
 * 
 * @warning SIMULATION ONLY - This backend is only compiled and active in SITL
 *          builds where CONFIG_HAL_BOARD == HAL_BOARD_SITL. It is not available
 *          on physical autopilot hardware and will not appear in production builds.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_FDM_ENABLED

#include "AP_RCProtocol_Backend.h"

/**
 * @class AP_RCProtocol_FDM
 * @brief RC protocol backend for SITL flight dynamics model simulation
 * 
 * @details AP_RCProtocol_FDM implements the RC input interface for Software-In-The-Loop
 *          simulation environments. Unlike hardware RC backends that decode physical
 *          serial protocols (SBUS, PPM, DSM, etc.), this backend receives RC channel
 *          values directly from the SITL simulation framework.
 * 
 *          The backend performs critical conversion between simulation coordinate
 *          systems and ArduPilot's RC input expectations:
 *          - Input: Normalized float values from simulator (-1.0 to +1.0)
 *          - Output: PWM microsecond values (1000-2000μs) matching physical RC
 * 
 *          Channel mapping follows standard ArduPilot RC assignments:
 *          - Channel 1: Roll (aileron)
 *          - Channel 2: Pitch (elevator)
 *          - Channel 3: Throttle
 *          - Channel 4: Yaw (rudder)
 *          - Channels 5-16: Auxiliary functions (mode switches, camera control, etc.)
 * 
 *          Simulation input sources:
 *          - Joystick/gamepad devices via SDL or native APIs
 *          - MAVProxy RC override commands (RC_CHANNELS_OVERRIDE)
 *          - Automated test scripts generating control sequences
 *          - External simulators (X-Plane, FlightGear, Gazebo) via interface protocols
 * 
 *          Conversion algorithm:
 *          For each channel, normalized simulator input is scaled to PWM:
 *          - Center value (0.0) → 1500μs (neutral position)
 *          - Minimum value (-1.0) → 1000μs (full down/left)
 *          - Maximum value (+1.0) → 2000μs (full up/right)
 *          - Linear interpolation: PWM = 1500 + (normalized * 500)
 * 
 *          Special considerations for throttle channel (typically channel 3):
 *          - Throttle uses unipolar range: 0.0 to +1.0
 *          - Maps to 1000μs (idle) through 2000μs (full throttle)
 * 
 * @note This backend is automatically selected by AP_RCProtocol when running
 *       in SITL mode. No manual backend selection is required.
 * 
 * @warning Simulation-only component - never active on physical hardware.
 *          All simulation RC inputs pass through this backend when in SITL.
 * 
 * @see AP_RCProtocol_Backend Base class defining RC backend interface
 * @see SITL::SIM Main SITL simulation class providing RC inputs
 */
class AP_RCProtocol_FDM : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Process RC input data from SITL flight dynamics model
     * 
     * @details This method is called periodically by the RC protocol manager to
     *          retrieve and process RC channel values from the SITL simulation
     *          framework. It reads normalized control inputs from the SITL::SIM
     *          class and converts them into standard PWM microsecond values that
     *          match physical RC receiver output.
     * 
     *          Update sequence:
     *          1. Query SITL::SIM for current joystick/scripted RC values
     *          2. Convert normalized inputs (-1.0 to +1.0) to PWM (1000-2000μs)
     *          3. Apply channel mapping from simulator axes to ArduPilot channels
     *          4. Store converted values in backend's RC channel array
     *          5. Update last_input_ms timestamp for activity tracking
     *          6. Mark RC input as valid via backend's add_input() method
     * 
     *          Conversion formula applied per channel:
     *          - For bipolar channels (roll, pitch, yaw): PWM = 1500 + (normalized * 500)
     *          - For throttle channel: PWM = 1000 + (normalized * 1000) where normalized is 0.0-1.0
     * 
     *          The method handles all 16 possible RC channels, though typically only
     *          8-12 channels are actively used depending on simulation configuration.
     * 
     *          Simulation input sources processed:
     *          - SDL joystick/gamepad raw axis values
     *          - MAVProxy RC_CHANNELS_OVERRIDE commands
     *          - Automated test scenario scripted inputs
     *          - External simulator interface protocols
     * 
     *          Call frequency: Invoked at main RC input processing rate (typically
     *          50-400Hz depending on AP_RCProtocol scheduler configuration)
     * 
     * @note This method is only active in SITL builds (CONFIG_HAL_BOARD == HAL_BOARD_SITL).
     *       It overrides the base class update() to provide simulation-specific behavior.
     * 
     * @warning Thread safety: Called from AP_RCProtocol thread context. SITL::SIM
     *          access is thread-safe via singleton pattern with internal locking.
     * 
     * @see AP_RCProtocol_Backend::add_input() Method to publish decoded RC values
     * @see SITL::SIM::get_rc_input() Source of normalized simulator RC inputs
     */
    void update() override;

    /**
     * @brief Check if FDM RC input is currently active
     * 
     * @details Determines whether this backend has received RC input data from the
     *          simulation recently. Active status indicates the SITL simulator is
     *          running and providing RC channel updates.
     * 
     *          Activity determination:
     *          - Active: last_input_ms > 0 (at least one update received since boot)
     *          - Inactive: last_input_ms == 0 (no simulation input received yet)
     * 
     *          This method is used by the RC protocol manager to select the appropriate
     *          backend when multiple RC sources might be available. In SITL mode, the
     *          FDM backend should always be active when the simulator is running.
     * 
     *          Unlike hardware RC backends that may timeout if signal is lost, the FDM
     *          backend considers itself active once any input has been received, as
     *          the simulator continuously provides RC values even if at neutral/zero.
     * 
     * @return true if simulation RC input has been received at least once
     * @return false if no simulation RC input received yet (simulator not started)
     * 
     * @note This is a lightweight check (single integer comparison) suitable for
     *       frequent polling by the RC protocol manager.
     * 
     * @see update() Method that updates last_input_ms timestamp
     */
    bool active() const { return last_input_ms > 0; }

private:

    /**
     * @brief Timestamp of last RC input received from SITL simulation
     * 
     * @details Stores the system time (in milliseconds since boot) when RC input
     *          was last received from the flight dynamics model. Used to determine
     *          backend activity status and track simulation health.
     * 
     *          Timestamp behavior:
     *          - Initial value: 0 (no input received yet)
     *          - Updated by: update() method on each successful RC data retrieval
     *          - Units: Milliseconds since system boot (AP_HAL::millis())
     * 
     *          Unlike hardware RC backends that use this timestamp for signal timeout
     *          detection, the FDM backend uses it primarily as an activity indicator.
     *          The simulation continuously provides RC values, so timeout logic is
     *          typically not needed.
     * 
     *          Usage:
     *          - Checked by active() to determine if backend is operational
     *          - Could be extended for simulation health monitoring
     *          - Debug logging of RC input data rates
     * 
     * @note Value persists for entire SITL session - never reset to zero after
     *       first input unless autopilot is rebooted.
     */
    uint32_t last_input_ms;
};


#endif  // AP_RCPROTOCOL_FDM_ENABLED
