/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_Mount_Backend_Serial.h
 * @brief Serial mount backend helper class for gimbal/mount control
 * 
 * This file defines the AP_Mount_Backend_Serial base class which provides
 * common serial communication infrastructure for mount backends that use
 * custom serial protocols. Derived classes implement specific protocol
 * handlers for various serial-based gimbal systems.
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include "AP_Mount_Backend.h"

/**
 * @class AP_Mount_Backend_Serial
 * @brief Base class for serial protocol-based gimbal mount backends
 * 
 * @details This class serves as a common base for mount backends that communicate
 *          with gimbals via custom serial protocols. It provides UART initialization
 *          and management infrastructure that is shared across multiple serial-based
 *          gimbal protocols.
 *          
 *          Derived serial protocol implementations include:
 *          - Alexmos (SimpleBGC)
 *          - CADDX
 *          - Siyi
 *          - SToRM32 (serial mode)
 *          - Topotek
 *          - Viewpro
 *          - Xacti
 *          
 *          The class handles UART connection setup via AP_SerialManager and provides
 *          common member variables for serial communication. Derived classes implement
 *          protocol-specific message parsing, command formatting, and control logic.
 * 
 * @note Thread-safety: UART operations should be called from the main scheduler thread.
 *       The init() method must be called before any serial communication attempts.
 */
class AP_Mount_Backend_Serial : public AP_Mount_Backend
{
public:
    /**
     * @brief Construct a new serial mount backend instance
     * 
     * @param[in] frontend      Reference to the AP_Mount frontend manager
     * @param[in] params        Reference to mount-specific parameters for this instance
     * @param[in] instance      Mount instance number (0-based index)
     * @param[in] serial_instance Serial port instance number for AP_SerialManager lookup
     */
    AP_Mount_Backend_Serial(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance, uint8_t serial_instance) :
        AP_Mount_Backend(frontend, params, instance),
        _serial_instance(serial_instance)
    {}

    /**
     * @brief Initialize the serial mount backend
     * 
     * @details Performs UART initialization by querying AP_SerialManager for the
     *          configured serial port matching _serial_instance. Sets up the UART
     *          connection and marks the backend as initialized if successful.
     *          
     *          This method must be called before attempting any serial communication
     *          with the gimbal. Derived classes may override to add protocol-specific
     *          initialization but should call this base implementation.
     */
    void init() override;

protected:

    // Internal member variables for serial communication
    
    /// Pointer to UART driver for serial communication with the gimbal
    /// Initialized by init() method via AP_SerialManager lookup
    AP_HAL::UARTDriver *_uart;
    
    /// Serial port instance number used for AP_SerialManager configuration lookup
    /// Corresponds to SERIALx_PROTOCOL parameter matching this mount's protocol
    uint8_t _serial_instance;
    
    /// Initialization status flag - true if UART has been successfully initialized
    /// Set by init() method, must be true before attempting serial communication
    bool _initialised;
};

#endif // HAL_MOUNT_ENABLED
