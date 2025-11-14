/**
 * @file SIMState.h
 * @brief Simulation state interface for Software-In-The-Loop (SITL) testing
 * 
 * Defines abstract interface for accessing simulation state during SITL runs.
 * Provides simulated sensor data, vehicle state, and environment conditions
 * for testing flight control algorithms without physical hardware.
 * 
 * SITL Architecture Overview:
 * - Physics Engine: JSBSim, X-Plane, Gazebo, or internal dynamics models
 * - Sensor Simulation: Adds realistic noise, latency, and errors to truth data
 * - Device Simulation: Simulates rangefinders, GPS, IMU, compass, and peripherals
 * - MAVLink Interface: Ground control station communication via UDP
 * - Failure Injection: Test failsafes with sensor faults, motor failures, wind
 * 
 * @note Only available in SITL builds (when AP_SIM_ENABLED is defined)
 * @note Stub implementation exists for non-SITL builds (returns null/zero)
 * @warning Do not use in production flight code - SITL testing only
 * 
 * @see SITL::Aircraft for physics simulation base class
 * @see SITL::SIM for simulation parameter configuration
 */

#pragma once

#include <SITL/SITL.h>

#if AP_SIM_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <SITL/SITL_Input.h>
#include <SITL/SIM_SoloGimbal.h>
#include <SITL/SIM_ADSB.h>
#include <SITL/SIM_Vicon.h>
#include <SITL/SIM_RF_Benewake_TF02.h>
#include <SITL/SIM_RF_Benewake_TF03.h>
#include <SITL/SIM_RF_Benewake_TFmini.h>
#include <SITL/SIM_RF_NoopLoop.h>
#include <SITL/SIM_RF_TeraRanger_Serial.h>
#include <SITL/SIM_RF_LightWareSerial.h>
#include <SITL/SIM_RF_LightWareSerialBinary.h>
#include <SITL/SIM_RF_Lanbao.h>
#include <SITL/SIM_RF_BLping.h>
#include <SITL/SIM_RF_LeddarOne.h>
#include <SITL/SIM_RF_RDS02UF.h>
#include <SITL/SIM_RF_USD1_v0.h>
#include <SITL/SIM_RF_USD1_v1.h>
#include <SITL/SIM_RF_MaxsonarSerialLV.h>
#include <SITL/SIM_RF_Wasp.h>
#include <SITL/SIM_RF_NMEA.h>
#include <SITL/SIM_RF_MAVLink.h>
#include <SITL/SIM_RF_GYUS42v2.h>
#include <SITL/SIM_VectorNav.h>
#include <SITL/SIM_MicroStrain.h>
#include <SITL/SIM_InertialLabs.h>
#include <SITL/SIM_AIS.h>
#include <SITL/SIM_GPS.h>

#include <SITL/SIM_EFI_Hirth.h>

#include <SITL/SIM_Frsky_D.h>
#include <SITL/SIM_CRSF.h>
#include <SITL/SIM_PS_RPLidarA2.h>
#include <SITL/SIM_PS_TeraRangerTower.h>
#include <SITL/SIM_PS_LightWare_SF45B.h>

#include <SITL/SIM_RichenPower.h>
#include <SITL/SIM_Loweheiser.h>
#include <SITL/SIM_FETtecOneWireESC.h>
#include <AP_HAL/utility/Socket_native.h>

#include <AP_HAL/AP_HAL_Namespace.h>

/**
 * @class AP_HAL::SIMState
 * @brief Simulation state manager for SITL (Software-In-The-Loop) testing
 * 
 * @details Manages simulation state and provides interface to simulated sensors,
 *          vehicle dynamics, and environment conditions during SITL execution.
 *          
 *          Primary responsibilities:
 *          - Sensor Simulation: GPS, IMU, barometer, compass, rangefinders
 *          - Analog Input Simulation: Airspeed, sonar, battery voltage/current
 *          - PWM I/O Simulation: RC inputs and servo/motor outputs
 *          - Device Simulation: Telemetry radios, gimbals, ADSB, peripherals
 *          - Environment Simulation: Wind, turbulence, temperature, air density
 *          - Truth Data Access: Actual position, velocity, attitude from physics
 *          - Failure Injection: Sensor faults, actuator failures, environmental extremes
 *          
 *          SITL Testing Use Cases:
 *          1. Algorithm Validation: Compare EKF estimates to physics truth data
 *          2. Automated Testing: Scripted missions with pass/fail criteria
 *          3. Failure Mode Testing: Inject faults, verify failsafe responses
 *          4. Integration Testing: Test MAVLink, parameter, mission protocols
 *          5. Developer Workflow: Rapid iteration without hardware risk
 *          
 *          Simulation Architecture:
 *          ```
 *          Physics Engine (JSBSim/X-Plane/Gazebo)
 *                  ↓
 *          SITL::Aircraft (dynamics model)
 *                  ↓
 *          AP_HAL::SIMState (sensor simulation)
 *                  ↓
 *          Vehicle Code (ArduCopter/Plane/etc.)
 *          ```
 *          
 *          Testing Workflow:
 *          - Launch: `sim_vehicle.py -v Copter --console --map`
 *          - Sensors read from SIMState instead of hardware
 *          - Physics engine updates vehicle state at each timestep
 *          - Sensor simulation adds realistic noise and latency
 *          - Control outputs sent to physics model
 *          - MAVLink telemetry connects to ground station
 *          
 * @note Physics fidelity varies by simulation backend
 * @note Timing is software-based, not real-time (can run faster/slower)
 * @note Only available when AP_SIM_ENABLED is defined
 * @warning Never include in production firmware builds
 * 
 * @see SITL::Aircraft Base class for vehicle physics simulation
 * @see SITL::SIM Configuration parameters for simulation behavior
 * @see AP_HAL_SITL HAL implementation for simulated hardware interfaces
 */
class AP_HAL::SIMState {
public:

    /**
     * @name Simulated Analog Sensor Pin Values
     * @brief Analog input values for simulated sensors
     * 
     * @details These values represent simulated analog inputs that would normally
     *          come from ADC pins on physical hardware. Updated by sensor simulation
     *          to provide realistic sensor readings with configurable noise and bias.
     *          
     *          Units: 16-bit ADC values (0-65535) scaled according to sensor type
     *          
     * @note Values updated in update() method each simulation timestep
     * @{
     */
    
    uint16_t sonar_pin_value;         ///< Simulated sonar/rangefinder analog output (pin 0)
    uint16_t airspeed_pin_value[2];   ///< Simulated airspeed sensor analog outputs (pin 1, dual sensors)
    uint16_t voltage_pin_value;       ///< Simulated battery voltage ADC reading (pin 13)
    uint16_t current_pin_value;       ///< Simulated battery current ADC reading (pin 12)
    uint16_t voltage2_pin_value;      ///< Simulated second battery voltage (pin 15)
    uint16_t current2_pin_value;      ///< Simulated second battery current (pin 14)
    
    /** @} */ // End of Simulated Analog Sensor Pin Values
    
    /**
     * @brief Update simulation state for current timestep
     * 
     * @details Called once per simulation loop iteration to:
     *          - Update physics model state from external simulator or internal dynamics
     *          - Generate simulated sensor readings with noise and latency
     *          - Update analog pin values (airspeed, sonar, battery)
     *          - Process RC inputs and update PWM outputs
     *          - Step device simulations (GPS, rangefinders, peripherals)
     *          - Handle network I/O (MAVLink, FDM data exchange)
     *          - Inject configured failures (sensor faults, actuator limits)
     *          
     *          Call frequency: Main loop rate (typically 400Hz for multicopters)
     *          
     * @note Must be called regularly to maintain simulation synchronization
     * @note Blocking calls may slow simulation execution
     * 
     * @see _fdm_input_step() Flight Dynamics Model input processing
     * @see _simulator_servos() PWM output processing
     */
    void update();

#if AP_SIM_GPS_ENABLED
    /**
     * @brief Set primary GPS simulation device
     * 
     * @details Assigns the GPS simulator object that will provide simulated
     *          GPS position, velocity, and satellite data for the primary GPS.
     *          Typically called during SITL initialization.
     *          
     * @param[in] _gps Pointer to GPS simulator instance
     * 
     * @note Only available when AP_SIM_GPS_ENABLED is defined
     * @see SITL::GPS GPS simulation class with configurable error models
     */
    void set_gps0(SITL::GPS *_gps) { gps[0] = _gps; }
#endif

    /**
     * @brief Simulated PWM output values
     * 
     * @details Array of PWM output values representing servo/motor commands
     *          sent to the physics simulation. These are the outputs from
     *          ArduPilot's motor mixing and servo control that would normally
     *          drive ESCs and servos on real hardware.
     *          
     *          Units: Microseconds (typically 1000-2000μs)
     *          Size: 32 channels (maximum supported output channels)
     *          
     *          Used by physics model to:
     *          - Calculate motor thrust and torque (multirotors)
     *          - Position control surfaces (fixed-wing)
     *          - Control steering/throttle (rovers)
     *          - Actuate thrusters (submarines)
     *          
     * @note Updated by _simulator_servos() each simulation step
     * @note Was previously SITL_NUM_CHANNELS constant
     */
    uint16_t pwm_output[32];  // was SITL_NUM_CHANNELS

private:
    /**
     * @name Internal Setup and Configuration
     * @{
     */
    
    void _set_param_default(const char *parm);     ///< Set default parameter value from string
    void _sitl_setup(const char *home_str);        ///< Initialize SITL with home position
    void _setup_timer(void);                       ///< Configure simulation timing
    void _setup_adc(void);                         ///< Initialize simulated ADC channels
    
    /** @} */
    
    /**
     * @name Simulation State Updates
     * @{
     */
    
    void set_height_agl(void);                     ///< Update height above ground level
    void _set_signal_handlers(void) const;         ///< Install signal handlers for clean shutdown
    
    /** @} */
    
    /**
     * @name Sensor and I/O Simulation
     * @{
     */
    
    void _update_airspeed(float airspeed);                    ///< Update simulated airspeed sensor reading
    void _simulator_servos(struct sitl_input &input);         ///< Process PWM outputs to physics model
    void _fdm_input_step(void);                               ///< Read Flight Dynamics Model input data
    void fdm_input_local(void);                               ///< Process locally-generated FDM data
    
    /** @} */
    
    /**
     * @brief Wait for simulation clock to advance
     * 
     * @details Synchronizes simulation timing by waiting until the specified
     *          simulation time has elapsed. Ensures consistent timesteps.
     *          
     * @param[in] wait_time_usec Target simulation time in microseconds
     */
    void wait_clock(uint64_t wait_time_usec);

    /**
     * @brief Simulated RC receiver PWM input values
     * 
     * @details Array of PWM input values from simulated RC receiver.
     *          These represent pilot stick commands and switch positions.
     *          
     *          Units: Microseconds (typically 1000-2000μs)
     *          Size: 16 channels (was SITL_RC_INPUT_CHANNELS)
     */
    uint16_t pwm_input[16];  // was SITL_RC_INPUT_CHANNELS

    /**
     * @name Internal State Variables
     * @{
     */
    
    // enum vehicle_type _vehicle;          ///< Vehicle type (commented out - unused)
    uint8_t _instance;                      ///< Instance number for multi-vehicle simulation
    uint16_t _base_port;                    ///< Base UDP port for MAVLink communication
    pid_t _parent_pid;                      ///< Process ID of parent simulator process
    uint32_t _update_count;                 ///< Count of update() calls for diagnostics
    
    /** @} */
    
    /**
     * @name Simulation Configuration and Parameters
     * @{
     */
    
    SITL::SIM *_sitl;                       ///< Simulation parameters and configuration object
    uint16_t _rcin_port;                    ///< UDP port for RC input injection
    uint16_t _fg_view_port;                 ///< UDP port for FlightGear visualization
    uint16_t _irlock_port;                  ///< UDP port for IR-Lock precision landing simulation
    float _current;                         ///< Simulated battery current draw (Amps)
    
    bool _use_rtscts;                       ///< Enable RTS/CTS flow control on simulated UARTs
    bool _use_fg_view;                      ///< Enable FlightGear external visualization
    
    const char *_fg_address;                ///< FlightGear network address for visualization
    
    /** @} */
    
    /**
     * @brief Physics simulation model
     * 
     * @details Pointer to vehicle-specific physics model that computes:
     *          - Aerodynamic forces and moments
     *          - Motor/propeller thrust and torque
     *          - 6-DOF rigid body dynamics integration
     *          - Ground contact and collision detection
     *          
     *          Derived from SITL::Aircraft base class. Concrete types include:
     *          - MultiCopter: Multirotor dynamics
     *          - Plane: Fixed-wing aerodynamics
     *          - QuadPlane: VTOL hybrid dynamics
     *          - Rover: Ground vehicle kinematics
     *          - Submarine: Underwater vehicle dynamics
     */
    SITL::Aircraft *sitl_model;

#if AP_SIM_SOLOGIMBAL_ENABLED
    /**
     * @name Simulated Gimbal System
     * @brief 3DR Solo gimbal simulation for camera stabilization testing
     * @{
     */
    bool enable_gimbal;                     ///< Enable/disable gimbal simulation
    SITL::SoloGimbal *gimbal;               ///< Solo gimbal simulation object
    /** @} */
#endif

#if AP_SIM_ADSB_ENABLED
    /**
     * @brief Simulated ADS-B traffic receiver
     * 
     * @details Generates simulated aircraft traffic for ADS-B collision
     *          avoidance testing. Injects MAVLink ADSB_VEHICLE messages.
     */
    SITL::ADSB *adsb;
#endif  // AP_SIM_ADSB_ENABLED

    /**
     * @name Simulated Positioning Systems
     * @{
     */
    
    SITL::Vicon *vicon;                     ///< Vicon motion capture system simulation

    /** @} */ // End of Simulated Positioning Systems
    
    /**
     * @name Simulated Rangefinder/Lidar Devices
     * @brief Serial rangefinder simulation objects for distance measurement testing
     * @details Each simulator implements specific protocol (I2C, UART, CAN, etc.)
     *          and device characteristics (range, accuracy, update rate, beam width)
     * @{
     */
    
    SITL::RF_Benewake_TF02 *benewake_tf02;           ///< Benewake TF02 lidar (UART, 12m range)
    SITL::RF_Benewake_TF03 *benewake_tf03;           ///< Benewake TF03 long-range lidar (UART, 180m)
    SITL::RF_Benewake_TFmini *benewake_tfmini;       ///< Benewake TFmini compact lidar (UART, 12m)
    SITL::RF_Nooploop *nooploop;                     ///< NoopLoop TOFSense lidar (UART)
    SITL::RF_TeraRanger_Serial *teraranger_serial;   ///< TeraRanger Serial lidar (I2C/UART)
    SITL::RF_LightWareSerial *lightwareserial;       ///< LightWare SF series legacy protocol (UART)
    SITL::RF_LightWareSerialBinary *lightwareserial_binary; ///< LightWare binary protocol (faster)
    SITL::RF_Lanbao *lanbao;                         ///< Lanbao PSK-CM8JL65-CC5 laser rangefinder
    SITL::RF_BLping *blping;                         ///< Blue Robotics Ping sonar (underwater)
    SITL::RF_LeddarOne *leddarone;                   ///< LeddarOne laser rangefinder (40m range)
    SITL::RF_RDS02UF *rds02uf;                       ///< RDS02UF ultrasonic sensor
    SITL::RF_USD1_v0 *USD1_v0;                       ///< USD1 ultrasonic v0 protocol
    SITL::RF_USD1_v1 *USD1_v1;                       ///< USD1 ultrasonic v1 protocol
    SITL::RF_MaxsonarSerialLV *maxsonarseriallv;     ///< MaxBotix MaxSonar-EZ series (UART)
    SITL::RF_Wasp *wasp;                             ///< Wasp 200 LRF laser rangefinder
    SITL::RF_NMEA *nmea;                             ///< NMEA-format rangefinder
    SITL::RF_MAVLink *rf_mavlink;                    ///< MAVLink-based rangefinder
    SITL::RF_GYUS42v2 *gyus42v2;                     ///< Grounding GYUS42v2 ultrasonic sensor
    
    /** @} */ // End of Simulated Rangefinder Devices

    /**
     * @name Simulated Telemetry Radio Systems
     * @{
     */
    
    SITL::Frsky_D *frsky_d;                          ///< FrSky D-series telemetry protocol
    // SITL::Frsky_SPort *frsky_sport;               ///< FrSky S.Port telemetry (planned)
    // SITL::Frsky_SPortPassthrough *frsky_sportpassthrough; ///< FrSky passthrough (planned)
    
    /** @} */
    
    /**
     * @name Simulated Proximity/Obstacle Detection Sensors
     * @brief 360-degree scanning sensors for collision avoidance
     * @{
     */

#if AP_SIM_PS_RPLIDARA2_ENABLED
    SITL::PS_RPLidarA2 *rplidara2;                   ///< RPLidar A2 360° laser scanner
#endif

#if AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
    SITL::PS_LightWare_SF45B *sf45b;                 ///< LightWare SF45B scanning lidar
#endif

#if AP_SIM_PS_TERARANGERTOWER_ENABLED
    SITL::PS_TeraRangerTower *terarangertower;       ///< TeraRanger Tower multi-sensor proximity
#endif
    
    /** @} */
    
    /**
     * @name Simulated ESC and Actuator Systems
     * @{
     */
    
    SITL::FETtecOneWireESC *fetteconewireesc;        ///< FETtec OneWire ESC telemetry protocol
    
    /** @} */

#if AP_SIM_CRSF_ENABLED
    /**
     * @brief Simulated CRSF (Crossfire) RC protocol
     * @details TBS Crossfire telemetry and RC link simulation
     */
    SITL::CRSF *crsf;
#endif

    /**
     * @name Simulated Inertial Navigation Systems (INS)
     * @brief High-accuracy external INS/AHRS simulation for testing sensor fusion
     * @{
     */
    
    SITL::VectorNav *vectornav;                      ///< VectorNav VN-100/200/300 INS
    SITL::MicroStrain5 *microstrain5;                ///< MicroStrain 3DM-GX5 series INS
    SITL::MicroStrain7 *microstrain7;                ///< MicroStrain 3DM-GX7 series INS
    SITL::InertialLabs *inertiallabs;                ///< InertialLabs INS-U INS/GPS system
    
    /** @} */ // End of Simulated INS

#if AP_SIM_JSON_MASTER_ENABLED
    /**
     * @brief JSON-based external physics simulation
     * @details Allows external simulators (Gazebo, Unity, Unreal Engine) to
     *          provide vehicle dynamics via JSON protocol over network socket
     */
    SITL::JSON_Master ride_along;
#endif  // AP_SIM_JSON_MASTER_ENABLED

#if AP_SIM_AIS_ENABLED
    /**
     * @brief Simulated AIS (Automatic Identification System)
     * @details Marine vessel tracking system simulation for autonomous boat testing
     */
    SITL::AIS *ais;
#endif

    /**
     * @name Simulated Electronic Fuel Injection (EFI) Systems
     * @brief Engine management system simulation for ICE-powered vehicles
     * @{
     */
    
    SITL::EFI_MegaSquirt *efi_ms;                    ///< MegaSquirt EFI controller simulation
    SITL::EFI_Hirth *efi_hirth;                      ///< Hirth 2-stroke EFI controller simulation
    
    /** @} */

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    /**
     * @brief FlightGear visualization output socket
     * @details UDP socket for sending vehicle state to FlightGear flight simulator
     *          for 3D visualization during SITL testing
     */
    SocketAPM_native fg_socket{true};
#endif

    /**
     * @name Configuration and Initialization
     * @{
     */
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;  ///< Path to parameter defaults file
    const char *_home_str;                                ///< Home position string (lat,lon,alt,yaw)
    uint32_t wind_start_delay_micros;                     ///< Delay before wind simulation starts (μs)
    
    /** @} */

#if AP_SIM_GPS_ENABLED
    /**
     * @brief Simulated GPS receiver devices
     * @details Array of GPS simulators for testing multi-GPS configurations.
     *          Each provides position, velocity, satellite count, HDOP/VDOP.
     *          
     *          Simulation features:
     *          - Configurable position error (horizontal/vertical)
     *          - GPS glitch injection for failsafe testing
     *          - GPS-denied environment simulation
     *          - RTK precision simulation
     *          - Satellite visibility modeling
     *          
     * @note Array size constrained by number of GPS parameter sets (typically 2)
     */
    SITL::GPS *gps[2];  // constrained by # of parameter sets
#endif
};

#endif // AP_SIM_ENABLED
