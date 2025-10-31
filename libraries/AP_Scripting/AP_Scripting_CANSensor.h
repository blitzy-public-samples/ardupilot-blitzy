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
 * @file AP_Scripting_CANSensor.h
 * @brief Scripting CAN sensor abstraction for Lua scripts to send and receive CAN frames
 * 
 * @details This file provides CANSensor-derived classes that enable Lua scripts to
 *          interact with the CAN bus. Scripts can send CAN frames, receive filtered
 *          CAN messages, and manage multiple independent receive buffers with different
 *          filter configurations.
 *          
 *          Key features:
 *          - Direct CAN frame transmission from Lua scripts
 *          - Buffered reception with configurable filtering (mask/value pairs)
 *          - Multiple independent buffers per CAN sensor instance
 *          - Thread-safe access using HAL semaphores
 *          
 *          The implementation uses a linked-list structure to support multiple
 *          consumer buffers, each with its own filter configuration and circular
 *          frame buffer.
 * 
 * @note This functionality is only available when HAL_MAX_CAN_PROTOCOL_DRIVERS > 0
 *       (or > 1 for AP_Periph builds)
 * 
 * @warning All frame handling and buffer operations are protected by semaphores.
 *          Callers must be aware of potential blocking when accessing shared resources.
 */
 
#pragma once

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_BUILD_AP_PERIPH)
    // Must have at least two CAN ports on Periph
    #define AP_SCRIPTING_CAN_SENSOR_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 1)
#else
    #define AP_SCRIPTING_CAN_SENSOR_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS
#endif

#if AP_SCRIPTING_CAN_SENSOR_ENABLED

#include <AP_CANManager/AP_CANSensor.h>

class ScriptingCANBuffer;

/**
 * @class ScriptingCANSensor
 * @brief CANSensor-derived class providing CAN bus access for Lua scripts
 * 
 * @details ScriptingCANSensor extends the CANSensor base class to provide a bridge
 *          between Lua scripting and the CAN bus hardware. It manages transmission
 *          of outgoing frames and distributes incoming frames to registered buffers
 *          for script consumption.
 *          
 *          Architecture:
 *          - Registers as a CAN protocol driver with specified protocol type
 *          - Maintains a linked list of ScriptingCANBuffer instances for frame distribution
 *          - Provides thread-safe frame transmission via write_frame()
 *          - Implements handle_frame() override to receive incoming CAN frames
 *          
 *          Lifecycle:
 *          1. Construct with desired AP_CAN::Protocol type (registers driver automatically)
 *          2. Create one or more ScriptingCANBuffer instances via add_buffer()
 *          3. Buffers receive frames via handle_frame() callback from CAN driver
 *          4. Scripts write frames via write_frame() and read via buffer read_frame()
 * 
 * @note Thread-safe: All operations protected by internal semaphore
 * @warning Multiple scripts sharing the same CAN sensor must coordinate to avoid conflicts
 * 
 * @see ScriptingCANBuffer for per-consumer buffering and filtering
 * @see CANSensor for base class functionality
 */
class ScriptingCANSensor : public CANSensor {
public:

    /**
     * @brief Construct a new Scripting CAN Sensor and register with CAN manager
     * 
     * @param[in] dtype CAN protocol type to register (e.g., AP_CAN::Protocol::Scripting)
     * 
     * @note Constructor automatically calls register_driver() to activate the sensor
     * @note Sensor name is set to "Script" for identification in logs and diagnostics
     */
    ScriptingCANSensor(AP_CAN::Protocol dtype)
        : CANSensor("Script") {
        register_driver(dtype);
    }

    /**
     * @brief Transmit a CAN frame on the bus with timeout
     * 
     * @details Writes a single CAN frame to the hardware CAN interface. This method
     *          provides the primary transmission path for Lua scripts to send CAN
     *          messages. The operation is protected by semaphore and will block up
     *          to the specified timeout waiting for bus availability.
     * 
     * @param[in] out_frame  CAN frame to transmit (contains ID, data length, and payload)
     * @param[in] timeout_us Maximum time to wait for transmission in microseconds
     * 
     * @return true if frame was successfully queued for transmission
     * @return false if timeout expired before frame could be sent
     * 
     * @warning This method may block for up to timeout_us microseconds
     * @note Frame priorities are managed by the underlying CAN driver
     * @note Extended CAN IDs (29-bit) are supported via frame.isExtended() flag
     * 
     * @see AP_HAL::CANFrame for frame structure details
     * @see CANSensor::send() for underlying transmission mechanism
     */
    bool write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us);

    /**
     * @brief Handle incoming CAN frame and distribute to buffers
     * 
     * @details Callback invoked by the CAN driver when a frame is received. Distributes
     *          the frame to all registered ScriptingCANBuffer instances by calling their
     *          handle_frame() methods. Each buffer applies its own filtering to determine
     *          whether to accept the frame.
     *          
     *          This method is called from the CAN driver's receive context and must
     *          complete quickly to avoid blocking other CAN operations.
     * 
     * @param[in] frame Received CAN frame to distribute to buffers
     * 
     * @note Thread-safe: Protected by internal semaphore
     * @warning Called from CAN driver context - must not perform blocking operations
     * @note Overrides CANSensor::handle_frame() pure virtual method
     * 
     * @see ScriptingCANBuffer::handle_frame() for per-buffer filtering
     */
    void handle_frame(AP_HAL::CANFrame &frame) override;

    /**
     * @brief Create and register a new buffer for frame reception
     * 
     * @details Allocates a new ScriptingCANBuffer with the specified size and adds it
     *          to the sensor's buffer list. Each buffer provides independent filtering
     *          and storage for received frames, allowing multiple scripts or consumers
     *          to receive different subsets of CAN traffic.
     *          
     *          Buffers are linked in a chain via the next pointer, and incoming frames
     *          are distributed to all buffers for filtering and potential storage.
     * 
     * @param[in] buffer_len Maximum number of frames the buffer can store (circular buffer size)
     * 
     * @return Pointer to newly created ScriptingCANBuffer on success
     * @return nullptr if memory allocation failed (NEW_NOTHROW returned null)
     * 
     * @note Uses NEW_NOTHROW for allocation - always check return value for nullptr
     * @note Buffers are not automatically freed - caller must manage lifetime
     * @warning Each buffer consumes memory proportional to buffer_len * sizeof(CANFrame)
     * 
     * @see ScriptingCANBuffer::add_filter() to configure frame filtering
     */
    ScriptingCANBuffer* add_buffer(uint32_t buffer_len);

private:

    /// Semaphore protecting access to buffer_list and frame operations
    HAL_Semaphore sem;

    /// Head of linked list of ScriptingCANBuffer instances for frame distribution
    ScriptingCANBuffer *buffer_list;

};

/**
 * @class ScriptingCANBuffer
 * @brief Per-consumer buffered CAN frame storage with configurable filtering
 * 
 * @details ScriptingCANBuffer provides an independent receive buffer for CAN frames
 *          with support for up to 8 configurable mask/value filter pairs. Each buffer
 *          maintains its own circular queue of received frames and can be used by
 *          separate Lua scripts or functions to receive specific CAN messages.
 *          
 *          Architecture:
 *          - ObjectBuffer provides circular frame storage (FIFO)
 *          - Filter array stores up to 8 mask/value pairs for selective reception
 *          - Linked-list chaining (next pointer) for multiple buffers per sensor
 *          - Reference to parent ScriptingCANSensor for frame transmission
 *          
 *          Filtering Mechanism:
 *          - Frames pass filter if: (frame.id & mask) == (value & mask)
 *          - Multiple filters are OR'ed (frame accepted if ANY filter matches)
 *          - Empty filter list accepts all frames
 *          - Filters apply to both standard (11-bit) and extended (29-bit) IDs
 *          
 *          Memory Allocation:
 *          - Created via ScriptingCANSensor::add_buffer() using NEW_NOTHROW
 *          - Buffer size specified at construction (number of frames)
 *          - Memory footprint: ~(buffer_size * sizeof(CANFrame)) + overhead
 *          
 *          Thread Safety:
 *          - All operations protected by internal semaphore
 *          - Safe for concurrent access from script and CAN driver contexts
 * 
 * @note Buffers are chained via linked list for distribution of incoming frames
 * @warning Filter operations and frame reads may block while acquiring semaphore
 * 
 * @see ScriptingCANSensor for parent sensor management
 * @see add_filter() for configuring frame acceptance criteria
 */
class ScriptingCANBuffer {
public:

    /**
     * @brief Construct a new Scripting CAN Buffer with specified capacity
     * 
     * @param[in] _sensor      Reference to parent ScriptingCANSensor for frame transmission
     * @param[in] buffer_size  Maximum number of CAN frames to store (circular buffer capacity)
     * 
     * @note Buffer is initially created with no filters (accepts all frames until filters added)
     * @note Actual memory allocation occurs in ObjectBuffer constructor
     */
    ScriptingCANBuffer(ScriptingCANSensor &_sensor, uint32_t buffer_size):
        buffer(buffer_size),
        sensor(_sensor)
    {};

    /**
     * @brief Transmit a CAN frame via parent sensor
     * 
     * @details Convenience method that forwards the frame to the parent
     *          ScriptingCANSensor::write_frame() for transmission. Allows scripts
     *          to transmit frames using either the sensor or buffer interface.
     * 
     * @param[in] out_frame  CAN frame to transmit
     * @param[in] timeout_us Maximum time to wait for transmission in microseconds
     * 
     * @return true if frame was successfully queued for transmission
     * @return false if timeout expired or transmission failed
     * 
     * @see ScriptingCANSensor::write_frame() for detailed transmission behavior
     */
    bool write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us);

    /**
     * @brief Read the oldest frame from the buffer (FIFO)
     * 
     * @details Retrieves and removes the next available frame from the circular buffer.
     *          If the buffer is empty, returns false immediately without blocking.
     *          Frames are returned in the order they were received (first-in, first-out).
     * 
     * @param[out] frame CAN frame structure to populate with received data
     * 
     * @return true if a frame was successfully read and removed from buffer
     * @return false if buffer is empty (no frames available)
     * 
     * @note Non-blocking: Returns immediately if no frames available
     * @note Thread-safe: Protected by internal semaphore
     * @warning Newer frames may be dropped if buffer fills before read_frame() is called
     * 
     * @see handle_frame() for frame insertion logic
     */
    bool read_frame(AP_HAL::CANFrame &frame);

    /**
     * @brief Process incoming frame and add to buffer if filters match
     * 
     * @details Applies configured filters to determine if the frame should be stored
     *          in this buffer. If filters match (or no filters configured), adds the
     *          frame to the circular buffer. Then recursively calls handle_frame() on
     *          the next buffer in the linked list to continue distribution.
     *          
     *          Filter Matching Logic:
     *          - If num_filters == 0: Accept all frames
     *          - If num_filters > 0: Accept if (frame.id & filter[i].mask) == (filter[i].value & filter[i].mask) for ANY i
     *          - Frame is dropped silently if buffer is full
     * 
     * @param[in] frame Received CAN frame to filter and potentially store
     * 
     * @note Thread-safe: Protected by internal semaphore
     * @note Recursive: Distributes frame to all buffers in linked list
     * @warning Called from CAN driver context - must complete quickly
     * @warning If buffer is full, frame is silently dropped (no error indication)
     * 
     * @see add_filter() for configuring frame acceptance criteria
     */
    void handle_frame(AP_HAL::CANFrame &frame);

    /**
     * @brief Recursively add a buffer to the end of the linked list
     * 
     * @details Traverses the linked list to find the last buffer (next == nullptr)
     *          and appends the new buffer. This maintains the chain of buffers that
     *          receive frames from the parent sensor's handle_frame() callback.
     * 
     * @param[in] new_buff Pointer to ScriptingCANBuffer to add to the chain
     * 
     * @note Recursive implementation traverses entire list to find insertion point
     * @note Thread-safe: Protected by internal semaphore
     * @warning O(n) complexity where n is number of existing buffers in chain
     */
    void add_buffer(ScriptingCANBuffer* new_buff);

    /**
     * @brief Add a mask/value filter pair for selective frame reception
     * 
     * @details Registers a new filter that determines which CAN frames are stored in
     *          this buffer. Supports up to 8 concurrent filters per buffer. Filters
     *          are applied with OR logic - a frame is accepted if it matches ANY filter.
     *          
     *          Filter Operation:
     *          - Frame accepted if: (frame.id & mask) == (value & mask)
     *          - Mask bits: 1 = compare this bit, 0 = ignore this bit
     *          - Value bits: expected value for compared bits
     *          
     *          Examples:
     *          - Accept ID 0x123 only: mask=0x7FF, value=0x123 (standard ID)
     *          - Accept IDs 0x100-0x1FF: mask=0x700, value=0x100 (filter on upper bits)
     *          - Accept extended ID 0x12345: mask=0x1FFFFFFF, value=0x12345
     * 
     * @param[in] mask  Bit mask specifying which bits of CAN ID to compare (1=compare, 0=ignore)
     * @param[in] value Expected value for masked bits (should have mask applied)
     * 
     * @return true if filter was successfully added
     * @return false if maximum number of filters (8) already configured
     * 
     * @note Multiple filters are OR'ed (frame accepted if ANY filter matches)
     * @note Empty filter list (num_filters == 0) accepts ALL frames
     * @note Filters apply to both standard (11-bit) and extended (29-bit) CAN IDs
     * @warning Maximum 8 filters per buffer - returns false if limit exceeded
     * 
     * @see handle_frame() for filter application logic
     */
    bool add_filter(uint32_t mask, uint32_t value);

private:

    /// Circular buffer for storing received CAN frames (FIFO queue)
    ObjectBuffer<AP_HAL::CANFrame> buffer;

    /// Reference to parent ScriptingCANSensor for frame transmission
    ScriptingCANSensor &sensor;

    /// Pointer to next buffer in linked list (nullptr if last in chain)
    ScriptingCANBuffer *next;

    /// Semaphore protecting buffer access and filter operations
    HAL_Semaphore sem;

    /**
     * @brief Filter configuration structure for CAN frame acceptance
     * 
     * @details Each filter defines a mask/value pair:
     *          - mask: Bits set to 1 are compared, bits set to 0 are ignored
     *          - value: Expected value for bits selected by mask
     *          Frame matches if: (frame.id & mask) == (value & mask)
     */
    struct {
        uint32_t mask;   ///< Bit mask for CAN ID comparison (1=compare, 0=ignore)
        uint32_t value;  ///< Expected value for masked bits
    } filter[8];         ///< Array of up to 8 filter configurations

    /// Number of active filters in filter[] array (0-8)
    uint8_t num_filters;

};

#endif // AP_SCRIPTING_CAN_SENSOR_ENABLED
