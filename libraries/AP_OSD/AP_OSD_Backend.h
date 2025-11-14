/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file AP_OSD_Backend.h
 * @brief Backend abstraction for ArduPilot On-Screen Display (OSD) subsystem
 * 
 * This file defines the abstract base class for OSD backends, providing a
 * hardware-independent interface for rendering text and symbols to various
 * OSD display devices. Concrete backend implementations (MSP, MAVLink, SITL)
 * inherit from this class and implement the pure virtual methods for their
 * specific hardware or protocol.
 * 
 * The backend architecture enables ArduPilot to support multiple OSD types
 * including analog video OSDs (via MAX7456 chips accessed through MSP protocol),
 * digital HD OSDs (via MAVLink camera protocol), and simulation displays (SITL).
 * 
 * Source: libraries/AP_OSD/AP_OSD_Backend.h
 */

#pragma once

#include <AP_HAL/HAL.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_Filesystem/AP_Filesystem.h>


/**
 * @class AP_OSD_Backend
 * @brief Abstract base class defining the interface contract for OSD backend implementations
 * 
 * @details This class provides the hardware abstraction layer for On-Screen Display
 *          rendering in ArduPilot. Concrete backend implementations must implement
 *          the pure virtual methods to support their specific display hardware or
 *          protocol.
 *          
 *          Backend Lifecycle:
 *          1. probe() - Derived class static method detects hardware presence
 *          2. Construction - Backend object created with AP_OSD reference
 *          3. init() - Initialize framebuffer and underlying hardware
 *          4. Rendering loop (called at display refresh rate):
 *             - clear() - Clear framebuffer and update blink phase
 *             - write() - Write text/symbols to framebuffer (multiple calls)
 *             - flush() - Push framebuffer contents to physical display
 *          5. Destruction - Cleanup resources
 *          
 *          The backend operates on a character-cell-based coordinate system where
 *          the display is treated as a grid of character positions. Typical display
 *          dimensions are 30 columns x 16 rows (PAL) or 30 columns x 13 rows (NTSC).
 *          
 *          Thread Safety: Backends are typically called from a dedicated OSD thread.
 *          Implementations must use appropriate semaphores when accessing shared
 *          resources (especially sensor data from AP singleton accessors).
 *          
 *          Symbol System: The OSD uses a custom symbol font where each 8-bit character
 *          code represents a specific icon or text character. Symbol constants (SYM_*)
 *          define standard icons for units, indicators, and display elements.
 * 
 * @note Rendering methods (write, flush, clear) are called at display refresh rate
 *       (typically 50Hz for PAL or 60Hz for NTSC), so implementations must be
 *       performance-conscious to avoid impacting flight control timing.
 * 
 * @warning Backends must be thread-safe when accessing shared ArduPilot data structures.
 *          Use WITH_SEMAPHORE or equivalent locking mechanisms when reading from
 *          AP::gps(), AP::ahrs(), AP::battery(), and other singleton accessors.
 * 
 * @see AP_OSD for the main OSD coordinator class
 * @see AP_OSD_MSP for analog video OSD backend (MAX7456 chips)
 * @see AP_OSD_MAVlink for digital HD OSD backend
 */
class AP_OSD_Backend
{

public:
    /**
     * @brief Construct OSD backend with reference to main OSD coordinator
     * 
     * @param[in] osd Reference to the AP_OSD coordinator object that manages
     *                this backend and provides access to configuration parameters
     */
    AP_OSD_Backend(AP_OSD &osd): _osd(osd) {}

    /**
     * @brief Virtual destructor for proper cleanup of derived backend classes
     */
    virtual ~AP_OSD_Backend(void) {}

    /**
     * @brief Write text string to OSD framebuffer at specified character position
     * 
     * @details This pure virtual method must be implemented by concrete backends
     *          to render text strings to their internal framebuffer. The text may
     *          contain regular ASCII characters or special symbol codes (SYM_*
     *          constants) for icons and display elements.
     *          
     *          Coordinate System: Character-cell positions, 0-indexed from top-left.
     *          Typical displays: 30 columns (x=0-29) × 16 rows (y=0-15) for PAL,
     *                           30 columns (x=0-29) × 13 rows (y=0-12) for NTSC.
     *          
     *          The text is written to an internal framebuffer and not immediately
     *          displayed. Call flush() to push framebuffer contents to the physical
     *          display hardware.
     * 
     * @param[in] x Horizontal character position (0-indexed, typically 0-29)
     * @param[in] y Vertical character position (0-indexed, typically 0-15 PAL or 0-12 NTSC)
     * @param[in] text Null-terminated string to display, may contain SYM_* codes
     * 
     * @note This method is called at display refresh rate (50-60Hz). Implementations
     *       must be efficient to avoid impacting flight control performance.
     * 
     * @warning Position coordinates exceeding display bounds should be handled
     *          gracefully (clipped or ignored) to prevent buffer overruns.
     */
    virtual void write(uint8_t x, uint8_t y, const char* text) = 0;

    /**
     * @brief Write formatted text to OSD framebuffer with optional blinking
     * 
     * @details This method provides printf-style formatted text rendering with
     *          support for blinking text. The blink parameter controls whether
     *          the text alternates between visible and invisible based on the
     *          current blink_phase counter (updated by clear()).
     *          
     *          Format string supports standard printf specifiers (%d, %f, %s, etc.)
     *          and backend-specific formatting extensions for optimized numeric
     *          display on character-cell displays.
     * 
     * @param[in] x Horizontal character position (0-indexed, typically 0-29)
     * @param[in] y Vertical character position (0-indexed, typically 0-15 PAL or 0-12 NTSC)
     * @param[in] blink true to make text blink at ~1.6Hz (phase changes every 4 frames),
     *                  false for static text
     * @param[in] fmt printf-style format string, may contain SYM_* codes
     * @param[in] ... Variable arguments matching format string specifiers
     * 
     * @note This method is called at display refresh rate (50-60Hz) during active
     *       panel rendering. Keep formatting operations efficient.
     */
    virtual void write(uint8_t x, uint8_t y, bool blink, const char *fmt, ...) FMT_PRINTF(5, 6);

    /**
     * @brief Initialize framebuffer and underlying hardware for OSD display
     * 
     * @details This pure virtual method is called once during backend startup to
     *          prepare the display hardware and allocate any required buffers.
     *          Implementations should:
     *          - Detect and configure display hardware (video format detection)
     *          - Allocate framebuffer memory
     *          - Initialize communication interfaces (SPI, UART, MAVLink channels)
     *          - Load font data if required
     *          - Configure display timing and synchronization
     *          
     *          Called during AP_OSD::init() after backend probe succeeds.
     * 
     * @return true if initialization successful and display ready for rendering,
     *         false if initialization failed (hardware not detected or error occurred)
     * 
     * @note Initialization failure typically results in OSD being disabled for the
     *       current flight. Some backends retry detection on subsequent boots.
     */
    virtual bool init() = 0;

    /**
     * @brief Push framebuffer contents to physical display hardware
     * 
     * @details This pure virtual method is called at the end of each rendering
     *          cycle (after all write() calls) to transfer the complete framebuffer
     *          to the display hardware. This is when text and symbols actually
     *          become visible to the user.
     *          
     *          Implementation typically involves:
     *          - For MSP/MAX7456: Sending character-by-character data over serial
     *          - For MAVLink: Encoding framebuffer into MAVLink camera messages
     *          - For SITL: Updating simulation display window
     *          
     *          The flush operation should be non-blocking or complete quickly to
     *          avoid impacting the OSD refresh rate and flight control timing.
     * 
     * @note This method is called at display refresh rate (50-60Hz). Implementations
     *       must complete within the frame time budget (~16-20ms) to maintain
     *       smooth display updates.
     * 
     * @warning Ensure framebuffer is in a consistent state before flush. All write()
     *          operations for the current frame must complete before flush() is called.
     */
    virtual void flush() = 0;

    /**
     * @brief Return aspect ratio correction factor for accurate angle display
     * 
     * @details Video displays may have non-square pixels depending on the video
     *          standard and display characteristics. This correction factor is
     *          applied to angular measurements (especially artificial horizon
     *          and directional indicators) to compensate for pixel aspect ratio
     *          and ensure angles appear geometrically correct.
     *          
     *          Typical values:
     *          - 1.0 (default): Square pixels, no correction needed
     *          - ~1.2: PAL displays with non-square pixel aspect ratio
     *          - ~1.0: NTSC displays (closer to square pixels)
     *          - Custom: HD displays with known pixel aspect ratios
     *          
     *          The correction factor is multiplied by horizontal angular measurements
     *          to compensate for pixel stretching/compression.
     * 
     * @return Aspect ratio correction factor (dimensionless multiplier)
     * 
     * @note Override this method in derived classes if the display hardware has
     *       non-square pixels requiring angular correction.
     */
    virtual float get_aspect_ratio_correction() const {return 1;}

    /**
     * @brief Clear framebuffer and advance blink phase counter
     * 
     * @details Prepares the framebuffer for the next rendering cycle by clearing
     *          previous content and updating the blink phase for blinking text.
     *          
     *          Blink Timing: The blink_phase counter cycles 0→1→2→3→0, changing
     *          once per frame. At 50Hz refresh rate, complete blink cycle is
     *          4 frames = 80ms (~12.5 blinks per second). At 60Hz, cycle is
     *          4 frames = 67ms (~15 blinks per second).
     *          
     *          Blinking text (write() with blink=true) is visible when
     *          (blink_phase & 2) == 0, creating a 50% duty cycle blink pattern.
     * 
     * @note This method is called at the start of each display refresh cycle,
     *       before any write() calls for the new frame.
     */
    virtual void clear()
    {
        blink_phase = (blink_phase+1)%4;
    }

    /**
     * @brief Copy backend-specific symbol set to OSD lookup table
     * 
     * @details Different OSD hardware may use different symbol character codes
     *          for the same visual icons. This method allows backends to remap
     *          the standard ArduPilot symbol codes (SYM_* constants) to their
     *          hardware-specific character codes.
     *          
     *          The base class provides a default symbol set in the symbols[]
     *          array. Backends can override specific symbols by copying their
     *          custom mappings to the provided lookup table.
     * 
     * @param[out] symbols Pointer to symbol lookup table to populate with
     *                     backend-specific character codes
     * @param[in] size Size of the symbols array in bytes (should match AP_OSD_NUM_SYMBOLS)
     * 
     * @note This is called during backend initialization to establish the
     *       symbol translation table used throughout rendering.
     */
    virtual void init_symbol_set(uint8_t *symbols, const uint8_t size);
    
    /**
     * @brief Check if this backend can operate with the specified OSD type configuration
     * 
     * @details Determines compatibility between backend implementation and the
     *          configured OSD type. This allows multiple backends to coexist
     *          and be selected based on user configuration parameters.
     * 
     * @param[in] type OSD type enumeration value from AP_OSD::osd_types
     * 
     * @return true if backend supports the specified OSD type,
     *         false if incompatible or unsupported
     * 
     * @note Used during backend probe to select appropriate backend based on
     *       OSD_TYPE parameter configuration.
     */
    virtual bool is_compatible_with_backend_type(AP_OSD::osd_types type) const = 0;
    
    /**
     * @brief Get the OSD type enumeration for this backend implementation
     * 
     * @details Returns the specific OSD type constant that identifies this
     *          backend (e.g., OSD_MSP, OSD_MAVLINK, OSD_SITL).
     * 
     * @return OSD type enumeration value from AP_OSD::osd_types identifying
     *         this backend's hardware/protocol type
     */
    virtual AP_OSD::osd_types get_backend_type() const = 0;

    /**
     * @brief Backend-specific processing called once per OSD thread iteration
     * 
     * @details Provides a hook for backends to perform periodic processing
     *          that needs to run on the OSD thread but outside the main
     *          clear→write→flush rendering cycle. Examples include:
     *          - Polling for hardware status changes
     *          - Processing incoming MAVLink messages
     *          - Updating backend-specific state
     *          
     *          Default implementation does nothing. Override in derived classes
     *          if periodic processing is required.
     * 
     * @note Called once per OSD thread loop iteration (typically 50-60Hz).
     *       Keep processing minimal to avoid impacting display refresh rate.
     */
    virtual void osd_thread_run_once() { return; }

    /**
     * @brief Get pointer to the parent AP_OSD coordinator object
     * 
     * @details Provides backends with access to the main OSD coordinator for
     *          reading configuration parameters and accessing shared OSD state.
     * 
     * @return Pointer to the AP_OSD coordinator object that owns this backend
     */
    AP_OSD * get_osd()
    {
        return &_osd;
    }

protected:
    /**
     * @brief Reference to parent AP_OSD coordinator object
     * 
     * @details Provides access to OSD configuration parameters, panel settings,
     *          and shared state. All backends maintain a reference to their
     *          parent coordinator throughout their lifetime.
     */
    AP_OSD& _osd;

    /**
     * @brief Get configured font number for display rendering
     * 
     * @details Returns the currently configured font selection from OSD parameters.
     *          Font number determines which character set is loaded and used for
     *          symbol rendering. Multiple fonts may be available on SD card or
     *          compiled into ROMFS.
     * 
     * @return Font number (0-based index), typically 0-3 depending on available fonts
     */
    uint8_t get_font_num(void) const
    {
        return (uint8_t)_osd.font_num.get();
    }

    /**
     * @brief Check if specific OSD option flag is enabled
     * 
     * @details Tests whether a particular option bit is set in the OSD options
     *          bitmask. Options control backend behavior such as units selection,
     *          imperial vs metric, text positioning, and feature enables.
     * 
     * @param[in] option Option bit mask to test (from AP_OSD option definitions)
     * 
     * @return true if option is enabled (bit set), false if disabled (bit clear)
     */
    bool check_option(uint32_t option) const
    {
        return (_osd.options & option) != 0;
    }

    /**
     * @brief Convert numeric string to decimal-packed character format
     * 
     * @details Some OSD hardware supports decimal-packed characters where digits
     *          and decimal points are combined into single character codes for
     *          more compact display. This method performs the conversion.
     * 
     * @param[in,out] buff Character buffer containing numeric string to convert
     * @param[in] size Size of buffer in bytes
     * 
     * @return Number of characters in packed result
     */
    uint8_t convert_to_decimal_packed_characters(char* buff, uint8_t size);
    
    /**
     * @brief Format string with printf-style arguments for OSD display
     * 
     * @details Backend-specific string formatting that handles printf format
     *          specifiers and optional decimal packing for optimized numeric
     *          display on character-cell hardware.
     * 
     * @param[out] dst Destination buffer for formatted string
     * @param[in] size Size of destination buffer in bytes
     * @param[in] decimal_packed true to apply decimal packing to numeric output,
     *                           false for standard ASCII formatting
     * @param[in] fmt printf-style format string
     * @param[in] ap Variable argument list matching format specifiers
     * 
     * @return Number of characters written to destination buffer (excluding null terminator)
     */
    virtual uint8_t format_string_for_osd(char* dst, uint8_t size, bool decimal_packed, const char *fmt, va_list ap);

    /**
     * @brief Load font character data from SD card or ROMFS
     * 
     * @details Reads font file containing character bitmaps for OSD symbol rendering.
     *          Fonts are searched first on SD card (for user customization), then
     *          in compiled ROMFS as fallback.
     * 
     * @param[in] font_num Font index to load (0-based)
     * 
     * @return Pointer to FileData containing font bitmap data, or nullptr if
     *         font could not be loaded
     * 
     * @note Caller is responsible for managing lifetime of returned FileData
     */
    FileData *load_font_data(uint8_t font_num);

    /**
     * @brief Current blink phase counter for blinking text
     * 
     * @details Cycles through values 0→1→2→3→0 on each clear() call. Blinking
     *          text visibility is determined by (blink_phase & 2), creating a
     *          50% duty cycle blink pattern at approximately 12.5Hz (PAL 50Hz
     *          refresh) or 15Hz (NTSC 60Hz refresh).
     */
    int8_t blink_phase;

    /**
     * @enum vid_format
     * @brief Video format enumeration for display timing
     * 
     * @details Identifies the video standard of the connected display, which
     *          determines resolution (PAL: 30×16 chars, NTSC: 30×13 chars)
     *          and refresh rate (PAL: 50Hz, NTSC: 60Hz).
     */
    enum vid_format {
        FORMAT_UNKNOWN = 0, ///< Video format not yet detected
        FORMAT_NTSC = 1,    ///< NTSC: 30×13 character display, 60Hz refresh
        FORMAT_PAL = 2,     ///< PAL: 30×16 character display, 50Hz refresh
    } _format;

    // ========================================================================
    // OSD Symbol Character Codes
    // ========================================================================
    // These constants define the character codes for OSD symbols in the
    // default ArduPilot font set. Each symbol represents a specific icon
    // or text character rendered on the display. Symbol codes are 8-bit
    // values (0x00-0xFF) that index into the OSD character font bitmap.
    //
    // Symbol Organization:
    // - Unit symbols: Distance, speed, electrical, temperature
    // - Status indicators: Battery, RSSI, arming, fence
    // - Navigation elements: Arrows, waypoint, home, wind
    // - Artificial horizon: Center markers, horizon bars, vertical bars
    // - Heading display: Cardinal directions, heading lines
    // - GPS indicators: Satellite, HDOP
    // - Attitude indicators: Roll, pitch symbols
    // - Sidebar elements: Progress bars and direction arrows
    // ========================================================================

    /**
     * @name Distance and Position Unit Symbols
     * @{
     */
    static const uint8_t SYM_M = 0xB9;      ///< Meters symbol 'm' for metric distance (code 0xB9)
    static const uint8_t SYM_KM = 0xBA;     ///< Kilometers symbol 'km' for metric distance (code 0xBA)
    static const uint8_t SYM_FT = 0x0F;     ///< Feet symbol 'ft' for imperial distance (code 0x0F)
    static const uint8_t SYM_MI = 0xBB;     ///< Miles symbol 'mi' for imperial distance (code 0xBB)
    static const uint8_t SYM_ALT_M = 0xB1;  ///< Altitude meters symbol with altitude icon (code 0xB1)
    static const uint8_t SYM_ALT_FT = 0xB3; ///< Altitude feet symbol with altitude icon (code 0xB3)
    static const uint8_t SYM_NM = 0xF1;     ///< Nautical miles symbol 'nm' (code 0xF1)
    /** @} */

    /**
     * @name Status and Battery Symbols
     * @{
     */
    static const uint8_t SYM_BATT_FULL = 0x90;    ///< Full battery icon (code 0x90)
    static const uint8_t SYM_BATT_UNKNOWN = 0x97; ///< Unknown battery status icon (code 0x97)
    static const uint8_t SYM_RSSI = 0x01;         ///< RSSI (signal strength) icon (code 0x01)
    /** @} */

    /**
     * @name Electrical Unit Symbols
     * @{
     */
    static const uint8_t SYM_VOLT = 0x06;  ///< Voltage symbol 'V' for volts (code 0x06)
    static const uint8_t SYM_AMP = 0x9A;   ///< Current symbol 'A' for amperes (code 0x9A)
    static const uint8_t SYM_MAH = 0x07;   ///< Milliamp-hour symbol 'mAh' for battery capacity (code 0x07)
    static const uint8_t SYM_WATT = 0xAE;  ///< Power symbol 'W' for watts (code 0xAE)
    static const uint8_t SYM_WH = 0xAB;    ///< Watt-hour symbol 'Wh' for energy (code 0xAB)
    static const uint8_t SYM_MW = 0xF4;    ///< Megawatt symbol 'MW' (code 0xF4)
    /** @} */

    /**
     * @name Speed Unit Symbols
     * @{
     */
    static const uint8_t SYM_MS = 0x9F;   ///< Meters per second symbol 'm/s' (code 0x9F)
    static const uint8_t SYM_FS = 0x99;   ///< Feet per second symbol 'ft/s' (code 0x99)
    static const uint8_t SYM_KMH = 0xA1;  ///< Kilometers per hour symbol 'km/h' (code 0xA1)
    static const uint8_t SYM_MPH = 0xB0;  ///< Miles per hour symbol 'mph' (code 0xB0)
    static const uint8_t SYM_KN = 0xF0;   ///< Knots symbol 'kn' for nautical speed (code 0xF0)
    static const uint8_t SYM_ASPD = 0xE1; ///< Airspeed indicator icon (code 0xE1)
    static const uint8_t SYM_GSPD = 0xE2; ///< Ground speed indicator icon (code 0xE2)
    static const uint8_t SYM_WSPD = 0xE3; ///< Wind speed indicator icon (code 0xE3)
    static const uint8_t SYM_VSPD = 0xE4; ///< Vertical speed indicator icon (code 0xE4)
    /** @} */

    /**
     * @name Angular and Rate Unit Symbols
     * @{
     */
    static const uint8_t SYM_DEGR = 0xA8;  ///< Degree symbol '°' for angles (code 0xA8)
    static const uint8_t SYM_RPM = 0xE0;   ///< RPM (revolutions per minute) symbol (code 0xE0)
    static const uint8_t SYM_DPS = 0xAA;   ///< Degrees per second symbol 'deg/s' (code 0xAA)
    /** @} */

    /**
     * @name General Unit and Indicator Symbols
     * @{
     */
    static const uint8_t SYM_PCNT = 0x25;    ///< Percent symbol '%' (code 0x25, ASCII %)
    static const uint8_t SYM_KILO = 0x4B;    ///< Kilo prefix 'K' (code 0x4B)
    static const uint8_t SYM_G = 0xDF;       ///< Gravity symbol 'G' for g-force (code 0xDF)
    static const uint8_t SYM_DIST = 0x22;    ///< Distance icon (code 0x22)
    /** @} */

    /**
     * @name Waypoint and Navigation Symbols
     * @{
     */
    static const uint8_t SYM_WPNO = 0xE5;  ///< Waypoint number icon (code 0xE5)
    static const uint8_t SYM_WPDIR = 0xE6; ///< Waypoint direction icon (code 0xE6)
    static const uint8_t SYM_WPDST = 0xE7; ///< Waypoint distance icon (code 0xE7)
    /** @} */

    /**
     * @name Time Unit Symbols
     * @{
     */
    static const uint8_t SYM_FTMIN = 0xE8; ///< Feet per minute symbol 'ft/min' for vertical speed (code 0xE8)
    static const uint8_t SYM_FTSEC = 0x99; ///< Feet per second symbol 'ft/s' (code 0x99, duplicate of SYM_FS)
    static const uint8_t SYM_CLK = 0xBC;   ///< Clock icon for time display (code 0xBC)
    static const uint8_t SYM_AH = 0xF3;    ///< Amp-hour symbol 'Ah' (code 0xF3)
    /** @} */

    /**
     * @name GPS and Positioning Symbols
     * @{
     */
    static const uint8_t SYM_SAT_L = 0x1E;    ///< Satellite icon left half (code 0x1E)
    static const uint8_t SYM_SAT_R = 0x1F;    ///< Satellite icon right half (code 0x1F)
    static const uint8_t SYM_HDOP_L = 0xBD;   ///< HDOP (horizontal dilution of precision) icon left (code 0xBD)
    static const uint8_t SYM_HDOP_R = 0xBE;   ///< HDOP icon right (code 0xBE)
    static const uint8_t SYM_GPS_LAT = 0xA6;  ///< GPS latitude icon (code 0xA6)
    static const uint8_t SYM_GPS_LONG = 0xA7; ///< GPS longitude icon (code 0xA7)
    /** @} */

    /**
     * @name Home and Environment Symbols
     * @{
     */
    static const uint8_t SYM_HOME = 0xBF;    ///< Home position icon (code 0xBF)
    static const uint8_t SYM_WIND = 0x16;    ///< Wind direction indicator icon (code 0x16)
    static const uint8_t SYM_TERALT = 0xEF;  ///< Terrain altitude icon (code 0xEF)
    static const uint8_t SYM_RNGFD = 0xF7;   ///< Rangefinder icon (code 0xF7)
    /** @} */

    /**
     * @name Direction Arrow Symbols
     * @{
     */
    static const uint8_t SYM_ARROW_START = 0x60; ///< First directional arrow (0°), base of 16-arrow sequence (code 0x60)
    static const uint8_t SYM_ARROW_COUNT = 16;   ///< Number of directional arrows (16 directions, 22.5° resolution)
    static const uint8_t SYM_ARROW_RIGHT = 0xFD; ///< Simple right arrow → (code 0xFD)
    static const uint8_t SYM_ARROW_LEFT = 0xFE;  ///< Simple left arrow ← (code 0xFE)
    /** @} */

    /**
     * @name Artificial Horizon Symbols
     * @{
     */
    static const uint8_t SYM_AH_H_START = 0x80;  ///< Artificial horizon horizontal bar start, base of 9-bar sequence (code 0x80)
    static const uint8_t SYM_AH_H_COUNT = 9;     ///< Number of horizontal horizon bars (9 positions for pitch indication)
    static const uint8_t SYM_AH_V_START = 0xCA;  ///< Artificial horizon vertical bar start, base of 6-bar sequence (code 0xCA)
    static const uint8_t SYM_AH_V_COUNT = 6;     ///< Number of vertical horizon bars (6 positions for roll indication)
    static const uint8_t SYM_AH_CENTER_LINE_LEFT = 0x26;  ///< Artificial horizon center reference line left half (code 0x26)
    static const uint8_t SYM_AH_CENTER_LINE_RIGHT = 0x27; ///< Artificial horizon center reference line right half (code 0x27)
    static const uint8_t SYM_AH_CENTER = 0x7E;   ///< Artificial horizon center marker (code 0x7E)
    /** @} */

    /**
     * @name Heading Display Symbols
     * @{
     */
    static const uint8_t SYM_HEADING_N = 0x18; ///< Cardinal direction North 'N' (code 0x18)
    static const uint8_t SYM_HEADING_S = 0x19; ///< Cardinal direction South 'S' (code 0x19)
    static const uint8_t SYM_HEADING_E = 0x1A; ///< Cardinal direction East 'E' (code 0x1A)
    static const uint8_t SYM_HEADING_W = 0x1B; ///< Cardinal direction West 'W' (code 0x1B)
    static const uint8_t SYM_HEADING_DIVIDED_LINE = 0x1C; ///< Heading scale tick mark (code 0x1C)
    static const uint8_t SYM_HEADING_LINE = 0x1D; ///< Heading scale line segment (code 0x1D)
    static const uint8_t SYM_HEADING = 0x89;   ///< Heading indicator icon (code 0x89)
    /** @} */

    /**
     * @name Vertical Motion Indicators
     * @{
     */
    static const uint8_t SYM_UP_UP = 0xA2;   ///< Rapid climb indicator (double up arrow) (code 0xA2)
    static const uint8_t SYM_UP = 0xA3;      ///< Climb indicator (single up arrow) (code 0xA3)
    static const uint8_t SYM_DOWN = 0xA4;    ///< Descent indicator (single down arrow) (code 0xA4)
    static const uint8_t SYM_DOWN_DOWN = 0xA5; ///< Rapid descent indicator (double down arrow) (code 0xA5)
    /** @} */

    /**
     * @name Temperature Unit Symbols
     * @{
     */
    static const uint8_t SYM_DEGREES_C = 0x0E; ///< Celsius symbol '°C' (code 0x0E)
    static const uint8_t SYM_DEGREES_F = 0x0D; ///< Fahrenheit symbol '°F' (code 0x0D)
    /** @} */

    /**
     * @name Arming and Safety Status Symbols
     * @{
     */
    static const uint8_t SYM_ARMED = 0x00;     ///< Vehicle armed indicator icon (code 0x00)
    static const uint8_t SYM_DISARMED = 0xE9;  ///< Vehicle disarmed indicator icon (code 0xE9)
    static const uint8_t SYM_FENCE_ENABLED = 0xF5;  ///< Geofence enabled icon (code 0xF5)
    static const uint8_t SYM_FENCE_DISABLED = 0xF6; ///< Geofence disabled icon (code 0xF6)
    static const uint8_t SYM_XERR = 0xEE;      ///< Error indicator 'X' (code 0xEE)
    /** @} */

    /**
     * @name Attitude Indicator Symbols
     * @{
     */
    static const uint8_t SYM_ROLL0 = 0x2D;     ///< Roll level indicator '-' (code 0x2D, ASCII dash)
    static const uint8_t SYM_ROLLR = 0xEA;     ///< Roll right indicator (code 0xEA)
    static const uint8_t SYM_ROLLL = 0xEB;     ///< Roll left indicator (code 0xEB)
    static const uint8_t SYM_ROLL = 0xA9;      ///< Roll angle icon (code 0xA9)
    static const uint8_t SYM_PTCH0 = 0x7C;     ///< Pitch level indicator '|' (code 0x7C, ASCII pipe)
    static const uint8_t SYM_PTCHUP = 0xEC;    ///< Pitch up indicator (code 0xEC)
    static const uint8_t SYM_PTCHDWN = 0xED;   ///< Pitch down indicator (code 0xED)
    static const uint8_t SYM_PITCH = 0xAF;     ///< Pitch angle icon (code 0xAF)
    /** @} */

    /**
     * @name Flight Status and Mode Symbols
     * @{
     */
    static const uint8_t SYM_FLY = 0x9C;    ///< Flying/airborne indicator icon (code 0x9C)
    static const uint8_t SYM_EFF = 0xF2;    ///< Efficiency icon (code 0xF2)
    static const uint8_t SYM_FLAP = 0x23;   ///< Flap position indicator (code 0x23)
    static const uint8_t SYM_RADIUS = 0x7A; ///< Turn radius icon (code 0x7A)
    /** @} */

    /**
     * @name Telemetry Link Quality Symbols
     * @{
     */
    static const uint8_t SYM_LQ = 0xF8;  ///< Link quality indicator icon (code 0xF8)
    static const uint8_t SYM_DB = 0xF9;  ///< Decibel symbol 'dB' (code 0xF9)
    static const uint8_t SYM_DBM = 0xFA; ///< dBm (decibel-milliwatts) symbol (code 0xFA)
    static const uint8_t SYM_SNR = 0xFB; ///< Signal-to-noise ratio symbol 'SNR' (code 0xFB)
    static const uint8_t SYM_ANT = 0xFC; ///< Antenna icon (code 0xFC)
    /** @} */

    /**
     * @name Sidebar Elements
     * @{
     */
    static const uint8_t SYM_SIDEBAR_R_ARROW = 0x09; ///< Sidebar right arrow (progress indicator) (code 0x09)
    static const uint8_t SYM_SIDEBAR_L_ARROW = 0x0A; ///< Sidebar left arrow (progress indicator) (code 0x0A)
    static const uint8_t SYM_SIDEBAR_A = 0x13; ///< Sidebar segment A (top of vertical bar) (code 0x13)
    static const uint8_t SYM_SIDEBAR_B = 0x14; ///< Sidebar segment B (code 0x14)
    static const uint8_t SYM_SIDEBAR_C = 0x15; ///< Sidebar segment C (code 0x15)
    static const uint8_t SYM_SIDEBAR_D = 0xDD; ///< Sidebar segment D (code 0xDD)
    static const uint8_t SYM_SIDEBAR_E = 0xDB; ///< Sidebar segment E (code 0xDB)
    static const uint8_t SYM_SIDEBAR_F = 0xDC; ///< Sidebar segment F (code 0xDC)
    static const uint8_t SYM_SIDEBAR_G = 0xDA; ///< Sidebar segment G (code 0xDA)
    static const uint8_t SYM_SIDEBAR_H = 0xDE; ///< Sidebar segment H (code 0xDE)
    static const uint8_t SYM_SIDEBAR_I = 0x11; ///< Sidebar segment I (code 0x11)
    static const uint8_t SYM_SIDEBAR_J = 0x12; ///< Sidebar segment J (bottom of vertical bar) (code 0x12)
    /** @} */

    /**
     * @brief Default OSD symbol lookup table
     * 
     * @details This array defines the default mapping from ArduPilot symbol indices
     *          to character codes in the OSD font. The array is indexed by symbol
     *          enumeration values and returns the corresponding 8-bit character code.
     *          
     *          Backends can customize symbol mappings by overriding init_symbol_set()
     *          to adapt to hardware-specific font character code assignments.
     *          
     *          Array size is AP_OSD_NUM_SYMBOLS elements, containing all standard
     *          ArduPilot OSD symbols in enumeration order.
     * 
     * @note This is a compile-time constant array used during backend initialization
     *       to populate the runtime symbol lookup table.
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
        SYM_SIDEBAR_R_ARROW,
        SYM_SIDEBAR_L_ARROW,
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
        SYM_WATT,
        SYM_WH,
        SYM_DB,
        SYM_DBM,
        SYM_SNR,
        SYM_ANT,
        SYM_ARROW_RIGHT,
        SYM_ARROW_LEFT,
        SYM_G,
        SYM_BATT_UNKNOWN,
        SYM_ROLL,
        SYM_PITCH,
        SYM_DPS,
        SYM_HEADING,
        SYM_RADIUS,
        SYM_FLAP,
    };
};
