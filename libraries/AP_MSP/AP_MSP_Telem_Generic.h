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
 * @file AP_MSP_Telem_Generic.h
 * @brief Generic MSP telemetry backend for standard MSP protocol clients
 * 
 * @details This backend implements the generic MSP (Multiwii Serial Protocol) telemetry
 *          interface for Betaflight, iNav, Cleanflight, and other standard MSP-compatible
 *          devices. Unlike vendor-specific backends (such as DisplayPort for OSDs),
 *          this backend provides minimal stateless request/response functionality without
 *          push telemetry scheduling.
 * 
 *          The generic backend is configured via SerialProtocol_MSP in AP_SerialManager
 *          and operates in a passive mode, responding to MSP queries from external devices
 *          without actively pushing telemetry data.
 * 
 * Source: libraries/AP_MSP/AP_MSP_Telem_Generic.h
 */
#pragma once

#include "AP_MSP_Telem_Backend.h"

#if HAL_MSP_ENABLED

/**
 * @class AP_MSP_Telem_Generic
 * @brief Generic MSP telemetry backend for Betaflight/iNav/Cleanflight-compatible devices
 * 
 * @details This minimal stateless backend disables the weighted fair queuing (WFQ) scheduler
 *          to operate in pure request/response mode. It responds to standard MSP queries
 *          from external devices without initiating telemetry transmissions.
 * 
 *          This backend is suitable for:
 *          - Basic MSP telemetry clients
 *          - Ground control stations using MSP protocol
 *          - Custom MSP-based monitoring tools
 *          - Debugging and development with MSP protocol analyzers
 * 
 *          Key characteristics:
 *          - No push telemetry (scheduler disabled)
 *          - Stateless operation
 *          - Request/response only
 *          - Standard MSP protocol compatibility
 * 
 * @note This backend is used for standard MSP protocol (SerialProtocol_MSP) without
 *       vendor-specific extensions. It inherits core MSP message handling from
 *       AP_MSP_Telem_Backend.
 * 
 * @warning This backend provides minimal functionality suitable for basic telemetry queries.
 *          For on-screen display (OSD) applications requiring active frame rendering and
 *          push telemetry, use AP_MSP_Telem_DisplayPort instead.
 * 
 * @see AP_MSP_Telem_Backend for core MSP message handling implementation
 * @see AP_MSP_Telem_DisplayPort for OSD-specific backend with push telemetry
 * @see AP_SerialManager::SerialProtocol for serial protocol configuration
 */
class AP_MSP_Telem_Generic : public AP_MSP_Telem_Backend
{
    /**
     * @note Constructor inheritance via using directive - inherits all constructors from
     *       AP_MSP_Telem_Backend without explicit redefinition. This provides the standard
     *       backend initialization with AP_HAL::UARTDriver reference.
     */
    using AP_MSP_Telem_Backend::AP_MSP_Telem_Backend;
public:
    /**
     * @brief Disable push telemetry scheduler for request/response operation
     * 
     * @details The generic MSP backend operates in passive request/response mode and does not
     *          require the weighted fair queuing (WFQ) scheduler used by other backends for
     *          active push telemetry. Disabling the scheduler prevents unnecessary processing
     *          overhead and ensures the backend only responds to incoming MSP queries.
     * 
     * @return false - Generic backend operates in request/response mode only without scheduler
     * 
     * @note Unlike the DJI backend which uses the scheduler for active push telemetry,
     *       the generic MSP backend doesn't push telemetry data; it only responds to queries
     *       from external MSP clients.
     * 
     * @see AP_MSP_Telem_Backend::is_scheduler_enabled() for base class virtual method
     */
    bool is_scheduler_enabled() const override { return false; }
    
    /**
     * @brief Get serial protocol type for generic MSP backend
     * 
     * @details Returns the serial protocol identifier used by AP_SerialManager to configure
     *          and route data to this backend. The SerialProtocol_MSP identifier indicates
     *          standard MSP protocol without vendor-specific extensions.
     * 
     * @return SerialProtocol_MSP for generic MSP devices
     * 
     * @note This identifies the backend in AP_SerialManager configuration. Users configure
     *       this protocol via SERIALx_PROTOCOL parameter to enable MSP telemetry on a
     *       specific UART port.
     * 
     * @see AP_SerialManager::SerialProtocol enumeration for all supported protocols
     * @see AP_MSP_Telem_Backend::get_serial_protocol() for base class virtual method
     */
    AP_SerialManager::SerialProtocol get_serial_protocol() const override { return AP_SerialManager::SerialProtocol::SerialProtocol_MSP; };
};

#endif //HAL_MSP_ENABLED
