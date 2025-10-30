#pragma once

/**
 * @file AP_CAN.h
 * @brief Common CAN protocol definitions shared across ArduPilot firmware variants
 * 
 * @details Authoritative source for CAN protocol type enumeration. Ensures consistent
 *          parameter values between main firmware and AP_Periph peripheral firmware.
 *          Intentionally minimal header (no AP_HAL dependencies) for maximum reusability.
 *          Not a catch-all CAN header - use specific headers (AP_CANManager.h,
 *          AP_CANDriver.h) for implementation.
 * 
 * @note Explicit numeric assignments ensure parameter stability across firmware versions
 * @warning Do not reuse deprecated protocol numbers - breaks parameter compatibility
 * 
 * @see AP_CANManager.h for runtime protocol driver management
 */

/*
 * this header contains data common to ArduPilot CAN, rather than to a
 * specific implementation of the protocols.  So we try to share
 * enumeration values where possible to make parameters similar across
 * Periph and main firmwares, for example.
 *
 * this is *not* to be a one-stop-shop for including all things CAN...
 * 
 * @note Shared enumeration values maintain parameter compatibility across builds
 * @see AP_CANManager.h for runtime protocol driver management
 */

#include <stdint.h>

/**
 * @class AP_CAN
 * @brief Namespace class for CAN protocol type enumeration
 * 
 * @details Defines Protocol enum used by CAN_Dn_PROTOCOL parameters. Not instantiated -
 *          serves as namespace for common definitions. Explicit uint8_t numeric assignments
 *          ensure ground station parameter compatibility and persistence stability.
 * 
 * @note Class exists only to provide namespace scoping - no instances created
 */
class AP_CAN {
public:
    /**
     * @enum AP_CAN::Protocol
     * @brief CAN protocol type identifiers for driver selection
     * 
     * @details Used by CAN_Dn_PROTOCOL and CAN_Dn_PROTOCOL2 parameters to specify which
     *          protocol driver handles a given CAN driver slot. Explicit numeric assignments
     *          preserve parameter storage across firmware versions.
     *          
     *          Deprecated values are marked with comments - do not reuse to avoid confusion
     *          when users upgrade from old firmware. Protocol drivers are conditionally
     *          compiled based on HAL_* feature flags. None (0) disables driver slot.
     *          
     *          Multiple drivers can use the same physical interface. CAN_Pn_DRIVER maps
     *          interface→driver index, CAN_Dn_PROTOCOL maps driver→protocol. Both must be
     *          configured correctly.
     * 
     * @note Parameter value in EEPROM/flash must match enum value - do not change assignments
     * @warning Reusing deprecated numbers breaks users upgrading from old firmware
     * @warning Multiple drivers can use same physical interface - CAN_Pn_DRIVER and
     *          CAN_Dn_PROTOCOL must both be configured correctly
     * 
     * @note To add new protocol: (1) Assign next available number, (2) Add conditional driver
     *       instantiation in AP_CANManager.cpp, (3) Implement AP_CANDriver subclass,
     *       (4) Update GCS parameter metadata
     * 
     * @see AP_CANManager.h for driver management implementation
     */
    enum class Protocol : uint8_t {
        /**
         * @brief No protocol driver assigned to this slot (driver disabled)
         * @note Default value, disables CAN_Dn_PROTOCOL driver slot
         */
        None = 0,
        
        /**
         * @brief DroneCAN/UAVCAN protocol driver
         * @details Primary ArduPilot CAN protocol supporting GPS, compass, airspeed,
         *          rangefinder, ESCs, servos. Enables redundant sensor configurations
         *          and distributed system architecture. Requires libcanard submodule.
         * @see libraries/AP_DroneCAN/
         */
        DroneCAN = 1,
        
        /**
         * @brief Deprecated - do not reuse
         * @note Originally assigned to KDECAN, now moved to protocol 8
         */
        // 2 was KDECAN -- do not re-use
        
        /**
         * @brief Deprecated - do not reuse
         * @note ToshibaCAN ESC protocol removed from codebase
         */
        // 3 was ToshibaCAN -- do not re-use
        
        /**
         * @brief Piccolo CAN protocol for Currawong ECUs and actuators
         * @details Electronic control units (ECUs) for fuel injection, generator control.
         *          Actuator control for servos. Conditional compilation via
         *          HAL_PICCOLO_CAN_ENABLE.
         * @see libraries/AP_PiccoloCAN/
         */
        PiccoloCAN = 4,
        
        /**
         * @brief Deprecated - do not reuse
         * @note Test protocol removed, do not reassign
         */
        // 5 was CANTester
        
        /**
         * @brief Northwest UAV EFI (Electronic Fuel Injection) protocol
         * @details Communicates with NWUAV fuel injection controllers for combustion engines
         */
        EFI_NWPMU = 6,
        
        /**
         * @brief USD1 CAN protocol
         * @details Vendor-specific protocol implementation
         */
        USD1 = 7,
        
        /**
         * @brief KDE Direct CAN protocol for KDE brushless ESCs
         * @details High-power ESC protocol supporting telemetry and configuration.
         *          Previously protocol 2, moved to 8 to avoid conflicts.
         */
        KDECAN = 8,
        
        /**
         * @brief Deprecated - do not reuse
         */
        // 9 was MPPT_PacketDigital
        
        /**
         * @brief Lua scripting CAN interface (primary)
         * @details Enables Lua scripts to send/receive custom CAN frames.
         *          Requires AP_SCRIPTING_ENABLED.
         * @see libraries/AP_Scripting/, CAN scripting bindings
         */
        Scripting = 10,
        
        /**
         * @brief Benewake CAN protocol for TF series LiDAR rangefinders
         * @details Supports TFmini Plus CAN, TF02 CAN rangefinders and proximity sensors
         */
        Benewake = 11,
        
        /**
         * @brief Lua scripting CAN interface (secondary)
         * @details Second scripting instance for drivers needing multiple CAN protocols
         *          simultaneously
         */
        Scripting2 = 12,
        
        /**
         * @brief TOFSenseP CAN protocol for laser rangefinders
         */
        TOFSenseP = 13,
        
        /**
         * @brief CAN radar protocol for NanoRadar and Hexsoon radar sensors
         * @details Supports obstacle detection and velocity measurement radars
         */
        RadarCAN = 14,  // used by NanoRadar and Hexsoon
    };
};
