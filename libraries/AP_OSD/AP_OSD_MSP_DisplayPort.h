/**
 * @file AP_OSD_MSP_DisplayPort.h
 * @brief MSP DisplayPort backend for HD OSD via MSP protocol
 * 
 * @details This file implements the MSP DisplayPort protocol backend for ArduPilot's
 *          On-Screen Display (OSD) system. The DisplayPort protocol enables communication
 *          with HD OSD devices (such as DJI FPV goggles, FatShark HD, and other MSP-compatible
 *          displays) over the Multiwii Serial Protocol (MSP).
 *          
 *          The implementation supports multiple display resolutions:
 *          - SD mode: 30 columns x 16 rows
 *          - HD mode: 50 columns x 18 rows
 *          - HD+ mode: 60 columns x 22 rows
 *          
 *          Character rendering uses the INAV font character set, requiring translation
 *          from ArduPilot's internal symbol representation to INAV-compatible codes.
 *          
 *          Key features:
 *          - Font translation between ArduPilot and INAV character sets
 *          - Write batching optimization for improved MSP bus efficiency
 *          - Runtime symbol set selection and code-page handling
 *          - Support for decimal packing controlled by MSP option flags
 *          - Integration with AP_MSP telemetry backend discovery
 *          
 * @note This backend requires an AP_MSP telemetry backend to be configured and
 *       operational on a UART port before DisplayPort communication can begin.
 * 
 * @see AP_OSD_Backend
 * @see AP_MSP_Telem_Backend
 */

#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_MSP/AP_MSP.h>

#if HAL_WITH_MSP_DISPLAYPORT

#define DISPLAYPORT_WRITE_BUFFER_MAX_LEN 30

/**
 * @class AP_OSD_MSP_DisplayPort
 * @brief DisplayPort protocol implementation for MSP-based HD OSD systems
 * 
 * @details This class implements the DisplayPort protocol for communicating with
 *          HD OSD devices over MSP (Multiwii Serial Protocol). DisplayPort is a
 *          standardized protocol originally developed by INAV for transmitting
 *          OSD screen data to external display devices.
 *          
 *          The implementation handles:
 *          - Character translation from ArduPilot to INAV font sets
 *          - Multiple resolution support (SD, HD, HD+)
 *          - Frame buffering and batch transmission
 *          - UART initialization within OSD thread context
 *          - Aspect ratio corrections for different display modes
 *          - Backend compatibility checking with other OSD types
 *          
 *          Protocol Flow:
 *          1. Probe for available AP_MSP backend
 *          2. Initialize DisplayPort communication
 *          3. Clear screen and setup defaults
 *          4. Write text to framebuffer positions
 *          5. Flush framebuffer to physical display
 *          
 *          Thread Safety:
 *          - UART initialization occurs in osd_thread_run_once() to ensure
 *            proper thread context for serial port access
 *          - Write operations buffer data before transmission
 *          
 * @note INAV compatibility requires specific character code mappings maintained
 *       in the ap_to_inav_symbols_map when AP_MSP_INAV_FONTS_ENABLED is defined.
 * 
 * @warning This backend is not compatible with simultaneous OSD_MSP or
 *          OSD_MSP_DISPLAYPORT instances due to UART resource conflicts.
 */
class AP_OSD_MSP_DisplayPort : public AP_OSD_Backend
{
    using AP_OSD_Backend::AP_OSD_Backend;
public:
    /**
     * @brief Factory method to probe and create MSP DisplayPort backend instance
     * 
     * @details Searches for an available AP_MSP telemetry backend to use for
     *          DisplayPort communication. The MSP backend must be configured and
     *          initialized before DisplayPort can be established.
     *          
     *          Probe sequence:
     *          1. Query AP_MSP for available telemetry backend
     *          2. Verify backend supports DisplayPort protocol
     *          3. Create AP_OSD_MSP_DisplayPort instance if backend found
     *          4. Return nullptr if no compatible backend available
     * 
     * @param[in] osd Reference to AP_OSD instance that will own this backend
     * 
     * @return Pointer to newly created AP_OSD_MSP_DisplayPort backend, or nullptr if probe fails
     * 
     * @note This is typically called during OSD initialization to auto-detect MSP DisplayPort capability
     * @see AP_MSP_Telem_Backend
     */
    static AP_OSD_Backend *probe(AP_OSD &osd);

    /**
     * @brief Initialize DisplayPort communication and underlying hardware
     * 
     * @details Performs complete initialization of the MSP DisplayPort interface:
     *          - Verifies AP_MSP backend availability
     *          - Establishes DisplayPort protocol handshake
     *          - Queries display capabilities (resolution, features)
     *          - Sets up default display parameters
     *          - Clears the display framebuffer
     *          
     *          This method is called after probe() succeeds and before any write operations.
     * 
     * @return true if initialization successful and DisplayPort is ready for use
     * @return false if initialization fails (MSP backend unavailable, protocol error, or hardware fault)
     * 
     * @note Initialization may be deferred for UART setup; see osd_thread_run_once()
     * @warning Must be called before any write(), flush(), or clear() operations
     */
    bool init() override;

    /**
     * @brief Write text to framebuffer at specified screen position
     * 
     * @details Writes a text string to the internal framebuffer at the given (x, y)
     *          screen coordinates. Text is translated from ArduPilot's internal symbol
     *          representation to the display's character encoding.
     *          
     *          The text is buffered and not immediately sent to the display; flush()
     *          must be called to transmit buffered content to the physical screen.
     *          
     *          Coordinate system:
     *          - Origin (0, 0) is top-left corner
     *          - X increases rightward (columns)
     *          - Y increases downward (rows)
     *          - Valid ranges depend on display resolution (SD/HD/HD+)
     * 
     * @param[in] x Column position (0 to columns-1, typically 0-29 SD, 0-49 HD, 0-59 HD+)
     * @param[in] y Row position (0 to rows-1, typically 0-15 SD, 0-17 HD, 0-21 HD+)
     * @param[in] text Null-terminated string containing ArduPilot symbol codes to display
     * 
     * @note Text is clipped if it extends beyond screen boundaries
     * @note Actual display update occurs on next flush() call
     * @see flush()
     * @see write_INAV()
     */
    void write(uint8_t x, uint8_t y, const char* text) override;
    
    /**
     * @brief Write text to framebuffer using INAV font character codes directly
     * 
     * @details Similar to write() but accepts text already encoded in INAV font
     *          character codes, bypassing the ArduPilot-to-INAV translation step.
     *          This method provides a performance optimization when text is already
     *          in INAV format or when custom character codes need to be sent.
     *          
     *          Write batching optimization:
     *          Multiple consecutive write_INAV() calls to adjacent positions are
     *          automatically batched into single MSP messages to reduce protocol
     *          overhead and improve bus efficiency.
     * 
     * @param[in] x Column position (0 to columns-1)
     * @param[in] y Row position (0 to rows-1)
     * @param[in] text Null-terminated string containing INAV font character codes
     * 
     * @note Text must be pre-encoded in INAV character set
     * @note Batching applies when consecutive calls have adjacent coordinates
     * @see write()
     */
    void write_INAV(uint8_t x, uint8_t y, const char* text);

    /**
     * @brief Flush framebuffer contents to physical display
     * 
     * @details Transmits all buffered screen updates to the physical display device
     *          via the MSP DisplayPort protocol. This makes all write() and write_INAV()
     *          operations since the last flush() visible on screen.
     *          
     *          The flush operation:
     *          1. Packages buffered changes into MSP DisplayPort messages
     *          2. Transmits message sequence over configured UART
     *          3. Sends display refresh/draw command to OSD hardware
     *          4. Clears internal buffer state for next frame
     *          
     *          Flush is typically called once per OSD update cycle (e.g., 5-10 Hz)
     *          after all screen elements have been written to the framebuffer.
     * 
     * @note This is a blocking operation that waits for MSP transmission completion
     * @note Call frequency affects display update rate and MSP bus utilization
     */
    void flush() override;

    /**
     * @brief Clear entire display framebuffer
     * 
     * @details Erases all content from the framebuffer, setting all screen positions
     *          to blank/space characters. The clear operation is buffered like write()
     *          and requires flush() to take effect on the physical display.
     *          
     *          Clear is typically called at the start of each OSD update cycle to
     *          remove previous frame content before drawing new elements.
     * 
     * @note Actual screen clearing occurs on next flush() call
     * @note More efficient than writing spaces to all positions individually
     */
    void clear() override;

    /**
     * @brief Initialize symbol lookup table with backend-specific character codes
     * 
     * @details Copies the MSP DisplayPort backend's symbol mapping into the provided
     *          lookup table, which ArduPilot uses to translate internal symbol IDs
     *          to display-specific character codes. This establishes the mapping from
     *          ArduPilot's AP_OSD_NUM_SYMBOLS enumeration to INAV font codes.
     *          
     *          Symbol set runtime selection:
     *          - Supports multiple code-pages for international characters
     *          - Handles font variations between display hardware
     *          - Maintains consistency with INAV character set standards
     *          
     *          The lookup table enables ArduPilot's generic OSD code to work with
     *          any backend by abstracting display-specific character encodings.
     * 
     * @param[out] lookup_table Pointer to symbol lookup table array to populate
     * @param[in]  size         Size of lookup_table array (must be >= AP_OSD_NUM_SYMBOLS)
     * 
     * @note Called during OSD initialization before any rendering occurs
     * @note lookup_table must remain valid for the backend's lifetime
     * @warning size must be sufficient to hold all AP_OSD_NUM_SYMBOLS entries
     */
    void init_symbol_set(uint8_t *lookup_table, const uint8_t size) override;

    /**
     * @brief One-time initialization within OSD thread context
     * 
     * @details Called exactly once by the OSD thread after thread creation but before
     *          normal OSD operations begin. This hook enables UART initialization to
     *          occur in the correct thread context, which is critical for some HAL
     *          implementations that require serial port registration in the using thread.
     *          
     *          UART reinitialization requirements:
     *          Some HAL platforms (e.g., ChibiOS) require UART registration to occur
     *          in the thread that will perform I/O operations. Since the MSP backend
     *          may be initialized in a different thread (e.g., main thread during boot),
     *          this method provides an opportunity to reinitialize or configure the
     *          UART in the OSD thread before OSD rendering begins.
     * 
     * @warning Must be called from OSD thread context only
     * @warning Do not perform blocking operations; this delays OSD startup
     * @note Called after init() but before first write/flush cycle
     * @see init()
     */
    void osd_thread_run_once() override;

    /**
     * @brief Get aspect ratio correction factor for angular displays
     * 
     * @details Returns a multiplicative correction factor to compensate for display
     *          aspect ratio when rendering angles and geometric elements. HD displays
     *          may have non-square pixels requiring correction to display circles as
     *          circles rather than ellipses, and angles at correct proportions.
     *          
     *          Typical correction factors:
     *          - SD (30x16, 4:3):  1.0 (no correction)
     *          - HD (50x18, 16:9): ~1.2 (compensate for wider aspect)
     *          - HD+ (60x22):      ~1.1 (varies by display)
     * 
     * @return Aspect ratio correction factor (dimensionless multiplier for X-axis scaling)
     * 
     * @note Applied to artificial horizon, compass rose, and other geometric OSD elements
     * @note Factor is determined from display resolution and pixel aspect ratio
     */
    float get_aspect_ratio_correction() const override;
    
    /**
     * @brief Check compatibility with other OSD backend types
     * 
     * @details Determines whether this MSP DisplayPort backend can coexist with
     *          another OSD backend type. This prevents resource conflicts when
     *          multiple OSD backends are configured.
     *          
     *          Compatibility rules:
     *          - NOT compatible with OSD_MSP or OSD_MSP_DISPLAYPORT (UART conflict)
     *          - Compatible with OSD_NONE, OSD_TXONLY (no resource overlap)
     *          - Compatible with OSD_MAX7456 (uses different SPI interface)
     *          - Compatible with OSD_SITL (simulation only)
     *          
     *          The primary conflict is UART resource sharing between multiple MSP
     *          backends attempting to use the same serial port.
     * 
     * @param[in] type OSD backend type to check for compatibility
     * 
     * @return true if this backend can coexist with the specified type
     * @return false if resource conflict would occur (incompatible)
     * 
     * @note Used during OSD initialization to validate multi-backend configurations
     * @warning Running incompatible backends simultaneously causes UART contention
     */
    bool is_compatible_with_backend_type(AP_OSD::osd_types type) const override {
        switch(type) {
        case AP_OSD::osd_types::OSD_MSP:
        case AP_OSD::osd_types::OSD_MSP_DISPLAYPORT:
            return false;
        case AP_OSD::osd_types::OSD_NONE:
        case AP_OSD::osd_types::OSD_TXONLY:
        case AP_OSD::osd_types::OSD_MAX7456:
        case AP_OSD::osd_types::OSD_SITL:
            return true;
        }
        return false;
    }

    /**
     * @brief Get the backend type identifier
     * 
     * @details Returns the OSD backend type enumeration value that identifies
     *          this as an MSP DisplayPort backend. Used by the OSD subsystem for
     *          backend identification, compatibility checking, and configuration.
     * 
     * @return AP_OSD::osd_types::OSD_MSP_DISPLAYPORT type identifier
     * 
     * @note Each backend implementation returns its unique type identifier
     */
    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_MSP_DISPLAYPORT;
    }
protected:
    /**
     * @brief Format string with OSD-specific decimal packing
     * 
     * @details Formats a string according to printf-style format specifiers with
     *          optional decimal packing controlled by MSP option flags. Decimal
     *          packing compresses floating point values by removing the decimal
     *          point character to save screen space on HD displays.
     *          
     *          Decimal packing behavior (when enabled):
     *          - "12.5" becomes "125" with implied decimal position
     *          - Controlled by MSP OPT_DECIMAL_PACK option flag
     *          - Display firmware interprets packed format based on context
     *          
     *          This optimization is particularly useful for HD displays where
     *          screen space is limited despite higher resolution.
     * 
     * @param[out] dst           Destination buffer for formatted string
     * @param[in]  size          Size of destination buffer in bytes (including null terminator)
     * @param[in]  decimal_packed True to enable decimal packing, false for normal formatting
     * @param[in]  fmt           Printf-style format string
     * @param[in]  ap            Variable argument list for format string
     * 
     * @return Number of characters written to dst (excluding null terminator)
     * 
     * @note Decimal packing is negotiated during DisplayPort initialization
     * @note dst is always null-terminated if size > 0
     * @warning Buffer overflow protection: output is truncated to size-1 characters
     */
    uint8_t format_string_for_osd(char* dst, uint8_t size, bool decimal_packed, const char *fmt, va_list ap) override;

private:
    /**
     * @brief Configure default display parameters
     * 
     * @details Sets up default configuration for the DisplayPort connection:
     *          - Default brightness and contrast levels
     *          - Blink rate for flashing elements
     *          - Character spacing and positioning offsets
     *          - Display mode preferences (SD/HD selection if auto-detect unavailable)
     *          
     *          Called during init() to establish known-good baseline settings
     *          before user-configured parameters are applied.
     * 
     * @note Internal method, not exposed to OSD subsystem
     */
    void setup_defaults(void);
    
    /**
     * @brief Internal buffer for batched write operations
     * 
     * @details Temporary buffer used to batch multiple character writes into single
     *          MSP messages for improved bus efficiency. The buffer accumulates
     *          characters from consecutive write_INAV() calls to adjacent screen
     *          positions before transmission.
     *          
     *          Buffer size (DISPLAYPORT_WRITE_BUFFER_MAX_LEN) limits maximum
     *          characters per batched message to balance efficiency and latency.
     */
    char displayport_write_buffer[DISPLAYPORT_WRITE_BUFFER_MAX_LEN]; // terminator

    /**
     * @brief Pointer to AP_MSP telemetry backend for DisplayPort communication
     * 
     * @details Reference to the AP_MSP backend that provides the underlying UART
     *          communication channel for DisplayPort protocol messages. Established
     *          during probe() and used for all MSP message transmission.
     *          
     *          Null if no compatible MSP backend is available.
     */
    AP_MSP_Telem_Backend* _displayport;

    /**
     * @name MSP DisplayPort Symbol Constants
     * @brief INAV font character codes for OSD symbols
     * 
     * @details Character code definitions for the INAV font character set used by
     *          MSP DisplayPort devices. These codes map to graphical symbols in the
     *          display's font ROM, representing units, indicators, and OSD elements.
     *          
     *          The INAV font set is standardized across compatible HD OSD devices
     *          (DJI, FatShark, etc.) providing consistent symbol rendering. Each
     *          SYM_* constant contains the byte value that renders the corresponding
     *          symbol when sent via DisplayPort protocol.
     *          
     *          Symbol categories:
     *          - Units: meters, feet, speed, altitude, voltage, current, etc.
     *          - Navigation: arrows, waypoints, home direction, wind
     *          - Attitude: artificial horizon, roll/pitch indicators
     *          - Status: armed/disarmed, GPS, RSSI, battery
     *          - Heading: compass points and heading line segments
     *          
     * @note These codes are INAV-specific and differ from MAX7456 character codes
     * @note Symbol appearance depends on font installed in display device firmware
     * @{
     */
    static const uint8_t SYM_M = 0x0C;
    static const uint8_t SYM_KM = 0x7D;
    static const uint8_t SYM_FT = 0x0F;
    static const uint8_t SYM_MI = 0x7E;
    static const uint8_t SYM_ALT_M = 0x0C;
    static const uint8_t SYM_ALT_FT = 0x0F;
    static const uint8_t SYM_BATT_FULL = 0x90;
    static const uint8_t SYM_RSSI = 0x01;

    static const uint8_t SYM_VOLT = 0x06;
    static const uint8_t SYM_AMP = 0x9A;
    static const uint8_t SYM_MAH = 0x07;
    static const uint8_t SYM_MS = 0x9F;
    static const uint8_t SYM_FS = 0x99;
    static const uint8_t SYM_KMH = 0x9E;
    static const uint8_t SYM_MPH = 0x9D;
    static const uint8_t SYM_DEGR = 0x08;
    static const uint8_t SYM_PCNT = 0x25;
    static const uint8_t SYM_RPM = 0x12;
    static const uint8_t SYM_ASPD = 0x41;
    static const uint8_t SYM_GSPD = 0x47;
    static const uint8_t SYM_WSPD = 0x57;
    static const uint8_t SYM_VSPD = 0x5E;
    static const uint8_t SYM_WPNO = 0x23;
    static const uint8_t SYM_WPDIR = 0xE6;
    static const uint8_t SYM_WPDST = 0xE7;
    static const uint8_t SYM_FTMIN = 0xE8;
    static const uint8_t SYM_FTSEC = 0x99;

    static const uint8_t SYM_SAT_L = 0x1E;
    static const uint8_t SYM_SAT_R = 0x1F;
    static const uint8_t SYM_HDOP_L = 0x48;
    static const uint8_t SYM_HDOP_R = 0x44;

    static const uint8_t SYM_HOME = 0x11;
    static const uint8_t SYM_WIND = 0x57;

    static const uint8_t SYM_ARROW_START = 0x60;
    static const uint8_t SYM_ARROW_COUNT = 16;
    static const uint8_t SYM_AH_H_START = 0x80;
    static const uint8_t SYM_AH_H_COUNT = 9;

    static const uint8_t SYM_AH_V_START = 0x82;
    static const uint8_t SYM_AH_V_COUNT = 6;

    static const uint8_t SYM_AH_CENTER_LINE_LEFT = 0x84;
    static const uint8_t SYM_AH_CENTER_LINE_RIGHT = 0x84;
    static const uint8_t SYM_AH_CENTER = 0x2B;

    static const uint8_t SYM_HEADING_N = 0x18;
    static const uint8_t SYM_HEADING_S = 0x19;
    static const uint8_t SYM_HEADING_E = 0x1A;
    static const uint8_t SYM_HEADING_W = 0x1B;
    static const uint8_t SYM_HEADING_DIVIDED_LINE = 0x1C;
    static const uint8_t SYM_HEADING_LINE = 0x1D;

    static const uint8_t SYM_UP_UP = 0x68;
    static const uint8_t SYM_UP = 0x68;
    static const uint8_t SYM_DOWN = 0x60;
    static const uint8_t SYM_DOWN_DOWN = 0x60;

    static const uint8_t SYM_DEGREES_C = 0x0E;
    static const uint8_t SYM_DEGREES_F = 0x0D;
    static const uint8_t SYM_GPS_LAT = 0x89;
    static const uint8_t SYM_GPS_LONG = 0x98;
    static const uint8_t SYM_ARMED = 0x00;
    static const uint8_t SYM_DISARMED = 0x2A;
    static const uint8_t SYM_ROLL0 = 0x2D;
    static const uint8_t SYM_ROLLR = 0x64;
    static const uint8_t SYM_ROLLL = 0x6C;
    static const uint8_t SYM_PTCH0 = 0x7C;
    static const uint8_t SYM_PTCHUP = 0x68;
    static const uint8_t SYM_PTCHDWN = 0x60;
    static const uint8_t SYM_XERR = 0x21;
    static const uint8_t SYM_KN = 0xF0;
    static const uint8_t SYM_NM = 0xF1;
    static const uint8_t SYM_DIST = 0x04;
    static const uint8_t SYM_FLY = 0x9C;
    static const uint8_t SYM_EFF = 0xF2;
    static const uint8_t SYM_AH = 0xF3;
    static const uint8_t SYM_MW = 0xF4;
    static const uint8_t SYM_CLK = 0x08;
    static const uint8_t SYM_KILO = 0x4B;
    static const uint8_t SYM_TERALT = 0x7F;
    static const uint8_t SYM_FENCE_ENABLED = 0xF5;
    static const uint8_t SYM_FENCE_DISABLED = 0xF6;
    static const uint8_t SYM_RNGFD = 0x7F;
    static const uint8_t SYM_LQ = 0xF8;

    static const uint8_t SYM_SIDEBAR_L_ARROW = 0x02;
    static const uint8_t SYM_SIDEBAR_R_ARROW = 0x03;
    static const uint8_t SYM_SIDEBAR_A = 0x13;
    static const uint8_t SYM_SIDEBAR_B = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_C = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_D = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_E = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_F = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_G = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_H = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_I = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_J = SYM_SIDEBAR_A;
    /** @} */ // end of MSP DisplayPort Symbol Constants

    /**
     * @brief ArduPilot symbol ID to INAV character code mapping table
     * 
     * @details Static lookup table mapping ArduPilot's internal AP_OSD symbol IDs
     *          to INAV DisplayPort character codes. This array provides the translation
     *          layer enabling ArduPilot's generic OSD code to work with INAV-compatible
     *          displays without knowing display-specific character encodings.
     *          
     *          Array organization:
     *          - Indexed by AP_OSD symbol enumeration values
     *          - Each entry contains corresponding INAV font character code
     *          - Array size is AP_OSD_NUM_SYMBOLS (total symbol count)
     *          
     *          The mapping is loaded into the OSD subsystem's lookup table during
     *          init_symbol_set() and used for all subsequent text rendering.
     *          
     *          Symbol ID translation example:
     *          ArduPilot symbol AP_OSD_SYMBOL_ALTITUDE_M (index 0) maps to
     *          INAV character code SYM_M (0x0C) at symbols[0].
     * 
     * @note This is the definitive symbol set for MSP DisplayPort backend
     * @note Constexpr enables compile-time initialization and optimization
     * @see init_symbol_set()
     */
    static constexpr uint8_t symbols[AP_OSD_NUM_SYMBOLS] {
        SYM_M,
        SYM_KM,
        SYM_FT,
        SYM_MI,
        SYM_ALT_M,
        SYM_ALT_FT,
        SYM_BATT_FULL,
        SYM_RSSI,
        SYM_VOLT,
        SYM_AMP,
        SYM_MAH,
        SYM_MS,
        SYM_FS,
        SYM_KMH,
        SYM_MPH,
        SYM_DEGR,
        SYM_PCNT,
        SYM_RPM,
        SYM_ASPD,
        SYM_GSPD,
        SYM_WSPD,
        SYM_VSPD,
        SYM_WPNO,
        SYM_WPDIR,
        SYM_WPDST,
        SYM_FTMIN,
        SYM_FTSEC,
        SYM_SAT_L,
        SYM_SAT_R,
        SYM_HDOP_L,
        SYM_HDOP_R,
        SYM_HOME,
        SYM_WIND,
        SYM_ARROW_START,
        SYM_ARROW_COUNT,
        SYM_AH_H_START,
        SYM_AH_H_COUNT,
        SYM_AH_V_START,
        SYM_AH_V_COUNT,
        SYM_AH_CENTER_LINE_LEFT,
        SYM_AH_CENTER_LINE_RIGHT,
        SYM_AH_CENTER,
        SYM_HEADING_N,
        SYM_HEADING_S,
        SYM_HEADING_E,
        SYM_HEADING_W,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_UP_UP,
        SYM_UP,
        SYM_DOWN,
        SYM_DOWN_DOWN,
        SYM_DEGREES_C,
        SYM_DEGREES_F,
        SYM_GPS_LAT,
        SYM_GPS_LONG,
        SYM_ARMED,
        SYM_DISARMED,
        SYM_ROLL0,
        SYM_ROLLR,
        SYM_ROLLL,
        SYM_PTCH0,
        SYM_PTCHUP,
        SYM_PTCHDWN,
        SYM_XERR,
        SYM_KN,
        SYM_NM,
        SYM_DIST,
        SYM_FLY,
        SYM_EFF,
        SYM_AH,
        SYM_MW,
        SYM_CLK,
        SYM_KILO,
        SYM_TERALT,
        SYM_FENCE_ENABLED,
        SYM_FENCE_DISABLED,
        SYM_RNGFD,
        SYM_LQ,
        SYM_SIDEBAR_L_ARROW,
        SYM_SIDEBAR_R_ARROW,
        SYM_SIDEBAR_A,
        SYM_SIDEBAR_B,
        SYM_SIDEBAR_C,
        SYM_SIDEBAR_D,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_H,
        SYM_SIDEBAR_I,
        SYM_SIDEBAR_J,
    };
    
#if AP_MSP_INAV_FONTS_ENABLED
    /**
     * @brief Extended character translation map for ArduPilot to INAV fonts
     * 
     * @details Comprehensive 256-entry translation table mapping ArduPilot character
     *          codes to INAV font character codes when INAV font support is enabled.
     *          This extended mapping handles the complete ASCII range plus ArduPilot's
     *          special symbol extensions.
     *          
     *          Map structure:
     *          - ap_to_inav_symbols_map[ardupilot_char][0] = INAV character code
     *          - ap_to_inav_symbols_map[ardupilot_char][1] = Alternate/fallback code
     *          
     *          The two-element array per character supports:
     *          - Primary character mapping for normal rendering
     *          - Alternate character for bold/highlighted rendering variants
     *          - Fallback character when primary is unavailable
     *          
     *          This mapping enables full character set translation including:
     *          - Standard ASCII alphanumerics (0x20-0x7E)
     *          - Extended ASCII graphics characters (0x80-0xFF)
     *          - ArduPilot-specific symbols and indicators
     *          - INAV-specific icon substitutions
     *          
     *          Font translation allows ArduPilot to use its standard symbol IDs
     *          internally while presenting INAV-compatible characters to the display.
     * 
     * @note Only compiled when AP_MSP_INAV_FONTS_ENABLED is defined
     * @note Defined in AP_OSD_MSP_DisplayPort.cpp with complete 256-entry mappings
     * @see write_INAV()
     */
    static const uint8_t ap_to_inav_symbols_map[256][2];
#endif //AP_MSP_INAV_FONTS_ENABLED
    
    /**
     * @brief Blink state for flashing OSD elements
     * 
     * @details Tracks the current blink phase for elements configured to flash/blink
     *          on the display. Toggled periodically to create flashing visual effect
     *          for warnings, alerts, and highlighted information.
     *          
     *          - true: Elements rendered (blink ON phase)
     *          - false: Elements hidden (blink OFF phase)
     *          
     *          Typical blink rate is 1-2 Hz for good visibility without distraction.
     */
    bool _blink_on;
};
#endif
