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
 * @file AP_OSD_MAX7456.h
 * @brief MAX7456 hardware OSD backend implementation for ArduPilot
 * 
 * @details This file implements the hardware backend for the Maxim Integrated
 *          MAX7456 analog on-screen display (OSD) chip. The MAX7456 is a single-chip,
 *          monochrome on-screen display generator for NTSC and PAL video that overlays
 *          text and graphics onto composite video signals.
 *          
 *          Key hardware features supported:
 *          - SPI communication interface for control and data transfer
 *          - 256-character font storage in internal NVRAM
 *          - Automatic PAL/NTSC video format detection
 *          - 30 columns × 13 rows (NTSC) or 30 columns × 16 rows (PAL) character display
 *          - Configurable horizontal and vertical position offsets
 *          - Video signal presence detection
 *          
 *          This implementation optimizes SPI traffic by maintaining a shadow frame buffer
 *          that tracks the last state transferred to the hardware, updating only changed
 *          characters during flush operations. Font synchronization is performed on
 *          initialization and when font changes are detected.
 * 
 * @see AP_OSD_Backend for the abstract backend interface
 * @see AP_OSD for the OSD manager and frontend
 */

#pragma once

#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_Common/Bitmask.h>

#if HAL_WITH_OSD_BITMAP

/**
 * @class AP_OSD_MAX7456
 * @brief Hardware backend for MAX7456 analog OSD chip over SPI
 * 
 * @details The AP_OSD_MAX7456 class implements direct hardware control of the MAX7456
 *          on-screen display generator chip via SPI bus communication. This backend
 *          handles all low-level hardware operations including:
 *          
 *          Hardware Interface:
 *          - SPI device initialization and communication via AP_HAL::Device
 *          - Register read/write operations with proper timing
 *          - Interrupt-safe SPI transfers using device semaphores
 *          
 *          Video Signal Management:
 *          - Automatic detection of PAL (625 lines) vs NTSC (525 lines) video format
 *          - Signal presence detection with debouncing (VIDEO_SIGNAL_DEBOUNCE_MS)
 *          - Automatic reinitilization on video signal loss/recovery
 *          - Aspect ratio correction factors for different video formats
 *          
 *          Font Management:
 *          - Upload of 256-character font set to MAX7456 NVRAM on initialization
 *          - Character-by-character verification and update when font changes
 *          - Font data loaded from SD card or ROMFS filesystem
 *          
 *          Frame Buffer Optimization:
 *          - Double-buffered rendering with current frame and shadow frame
 *          - Shadow frame tracks last state transferred to hardware
 *          - Differential updates during flush() send only changed characters
 *          - Reduces SPI traffic and improves performance
 *          
 *          Display Characteristics:
 *          - Character grid: 30 columns (0-29) × variable rows based on video format
 *          - NTSC: 13 rows (0-12) at 525 lines/60Hz
 *          - PAL: 16 rows (0-15) at 625 lines/50Hz
 *          - Character size: Each character is 12×18 pixels
 *          
 * @warning This backend requires stable video signal (composite NTSC or PAL) on the
 *          MAX7456 video input pins. Video format detection requires several video
 *          frames to debounce, so display may not appear immediately after power-on.
 *          
 * @warning All SPI operations must be performed with the device semaphore held to
 *          ensure thread-safe access to the hardware.
 * 
 * @note The MAX7456 chip has built-in character generation and does not require
 *       pixel-level rendering. All text and symbols are rendered using the uploaded
 *       256-character font set stored in the chip's NVRAM.
 */
class AP_OSD_MAX7456 : public AP_OSD_Backend
{

public:

    /**
     * @brief Probe for MAX7456 chip presence and create backend instance
     * 
     * @details Factory method that attempts to detect and initialize a MAX7456 OSD chip
     *          on the provided SPI device. The probe sequence performs:
     *          1. Soft reset of MAX7456 via VM0 register
     *          2. Verification of register read/write functionality
     *          3. Initial video signal detection
     *          4. Creation of backend instance if hardware responds correctly
     *          
     *          This method is typically called during AP_OSD initialization when scanning
     *          for available OSD hardware backends. The SPI device is usually obtained
     *          from the HAL with a name like "osd" that maps to board-specific pins.
     * 
     * @param[in] osd Reference to the AP_OSD manager instance
     * @param[in] dev SPI device handle with ownership transfer (OwnPtr)
     * 
     * @return Pointer to new AP_OSD_MAX7456 instance if probe successful, nullptr otherwise
     * 
     * @note This method takes ownership of the device handle. If probe fails, the device
     *       handle is automatically released.
     * 
     * @warning The SPI device must be properly configured in the board's hardware
     *          definition (hwdef) with correct chip select, clock speed, and mode.
     * 
     * @see AP_OSD::init_backend()
     */
    static AP_OSD_Backend *probe(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Write text string to framebuffer at specified position
     * 
     * @details Writes a null-terminated text string into the local frame buffer at the
     *          specified character cell position. The text is NOT immediately sent to the
     *          MAX7456 hardware; it is staged in the local frame buffer until flush() is
     *          called. Characters are written left-to-right starting from the (x,y) position.
     *          
     *          The text string should contain character codes that correspond to the uploaded
     *          font set. Special symbols use codes defined in AP_OSD_Backend (SYM_* constants).
     *          Text that extends beyond the right edge of the display (column 29) is clipped.
     * 
     * @param[in] x Column position (0-29, left to right)
     * @param[in] y Row position (0-12 for NTSC, 0-15 for PAL, top to bottom)
     * @param[in] text Null-terminated string to display (character codes from font set)
     * 
     * @note This method only updates the local frame buffer. Call flush() to transfer
     *       changes to the hardware and make them visible on screen.
     * 
     * @note Position (0,0) is the top-left corner of the display. Coordinates outside
     *       the valid range are clipped.
     * 
     * @warning Text content is NOT validated. Ensure character codes exist in the loaded
     *          font or they will display as blank/incorrect characters.
     * 
     * @see flush()
     * @see clear()
     */
    void write(uint8_t x, uint8_t y, const char* text) override;

    /**
     * @brief Initialize MAX7456 hardware and upload font
     * 
     * @details Performs complete initialization sequence for the MAX7456 chip:
     *          1. Software reset via VM0 register (clear all settings)
     *          2. Video format detection (PAL vs NTSC) with debounce delay
     *          3. Determination of visible row count based on detected format
     *          4. Font upload from filesystem to MAX7456 NVRAM (256 characters)
     *          5. Configuration of display mode (enable OSD, set white level)
     *          6. Clear display memory to blank state
     *          
     *          Font data is loaded from:
     *          - SD card: /APM/Font/fontN.bin (where N is OSD_FONT parameter)
     *          - Fallback: ROMFS embedded default font
     *          
     *          Font upload takes several hundred milliseconds as each character requires
     *          multiple SPI transactions to write to NVRAM and verify.
     * 
     * @return true if initialization successful (chip responding, font uploaded)
     * @return false if initialization failed (no video signal, SPI errors, font load failure)
     * 
     * @note This method blocks during font upload. Called once at system startup by the
     *       OSD manager thread, not from interrupt context.
     * 
     * @warning Requires valid video signal (PAL or NTSC) on MAX7456 input. Initialization
     *          will retry internally with debounce timing if video signal is not immediately
     *          stable. Display will not function without video signal present.
     * 
     * @warning Font upload failure is critical - OSD will show garbage characters if font
     *          is not correctly loaded to NVRAM.
     * 
     * @see update_font()
     * @see reinit()
     */
    bool init() override;

    /**
     * @brief Transfer framebuffer changes to MAX7456 hardware
     * 
     * @details Performs differential update of the MAX7456 display memory by comparing
     *          the current frame buffer with the shadow frame (last transferred state).
     *          Only character positions that have changed are written to the hardware
     *          via SPI, significantly reducing bus traffic compared to full-frame updates.
     *          
     *          The flush operation:
     *          1. Acquires SPI device semaphore for thread-safe hardware access
     *          2. Iterates through all character positions comparing frame vs shadow_frame
     *          3. Builds batched SPI commands for changed characters using DMM auto-increment
     *          4. Transfers command buffer to MAX7456 via SPI
     *          5. Updates shadow_frame to match current frame state
     *          6. Releases semaphore
     *          
     *          The MAX7456 Display Memory Mode (DMM) auto-increment feature is used to
     *          efficiently write consecutive changed characters with reduced command overhead.
     * 
     * @note Called periodically by the OSD manager thread at the screen refresh rate
     *       (typically 10-20Hz). Must complete quickly to avoid frame drops.
     * 
     * @note If no characters have changed since last flush, this method returns quickly
     *       with minimal SPI traffic (efficient for static displays).
     * 
     * @warning This method must not be called from interrupt context. SPI semaphore
     *          acquisition may block briefly if another thread is accessing the bus.
     * 
     * @see write()
     * @see transfer_frame()
     * @see is_dirty()
     */
    void flush() override;

    /**
     * @brief Clear framebuffer to blank state
     * 
     * @details Clears the local frame buffer by filling all character positions with
     *          null/blank characters (0x00). This does NOT immediately clear the display;
     *          flush() must be called to transfer the blank frame to hardware.
     *          
     *          The shadow frame is NOT modified, so the next flush() will detect all
     *          positions as changed and send a full blank frame to the MAX7456.
     * 
     * @note This only clears the local buffer. Call flush() to make the blank screen
     *       visible on the display.
     * 
     * @note Clearing is fast (simple memset operation) but subsequent flush() will have
     *       maximum SPI traffic as all character positions are marked dirty.
     * 
     * @see flush()
     * @see write()
     */
    void clear() override;

    /**
     * @brief Return aspect ratio correction factor for current video format
     * 
     * @details The MAX7456 character cells are not square - they have different width
     *          and height in pixels. Additionally, PAL and NTSC have different aspect
     *          ratios. This method returns a correction factor used to display angular
     *          elements (like artificial horizon lines) with correct geometry.
     *          
     *          Typical correction factors:
     *          - NTSC: ~0.75 (characters are taller than wide)
     *          - PAL: ~0.83 (characters are taller than wide, different from NTSC)
     *          
     *          The correction factor is calculated based on:
     *          - Video standard pixel aspect ratio (NTSC: 525 lines, PAL: 625 lines)
     *          - MAX7456 character cell dimensions (12 pixels wide × 18 pixels tall)
     *          - Display active area dimensions
     * 
     * @return Aspect ratio correction factor (typically 0.7-0.9)
     * 
     * @note This value changes dynamically if video format switches between PAL and NTSC
     *       due to video signal change or user configuration.
     * 
     * @note Used by horizon, compass rose, and other geometric panel elements to render
     *       with correct proportions on screen.
     */
    float get_aspect_ratio_correction() const override;

    /**
     * @brief Check if this backend is compatible with specified OSD type
     * 
     * @details Determines whether this MAX7456 hardware backend can coexist with another
     *          OSD backend of the specified type. The MAX7456 backend is incompatible with
     *          other hardware OSD backends that attempt to drive the same display, but
     *          compatible with telemetry-only backends that don't drive displays.
     *          
     *          Compatibility rules:
     *          - Incompatible: OSD_MAX7456 (duplicate), OSD_SITL (simulation conflict)
     *          - Compatible: OSD_NONE, OSD_TXONLY (telemetry relay), OSD_MSP (external OSD),
     *                        OSD_MSP_DISPLAYPORT (external OSD)
     *          
     *          This allows configurations where MAX7456 renders locally while telemetry
     *          is simultaneously sent to ground station or external OSD devices.
     * 
     * @param[in] type OSD backend type to check compatibility against
     * 
     * @return true if backends can coexist, false if mutually exclusive
     * 
     * @note Used by AP_OSD manager when initializing multiple OSD backends to prevent
     *       conflicting hardware access.
     */
    bool is_compatible_with_backend_type(AP_OSD::osd_types type) const override {
        switch(type) {
        case AP_OSD::osd_types::OSD_MAX7456:
        case AP_OSD::osd_types::OSD_SITL:
            return false;
        case AP_OSD::osd_types::OSD_NONE:
        case AP_OSD::osd_types::OSD_TXONLY:
        case AP_OSD::osd_types::OSD_MSP:
        case AP_OSD::osd_types::OSD_MSP_DISPLAYPORT:
            return true;
        }
        return false;
    }

    /**
     * @brief Return the backend type identifier
     * 
     * @details Returns the OSD backend type enumeration value that identifies this
     *          backend as a MAX7456 hardware implementation. Used by the OSD manager
     *          for backend identification, compatibility checking, and logging.
     * 
     * @return AP_OSD::osd_types::OSD_MAX7456 type identifier
     */
    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_MAX7456;
    }

private:

    /**
     * @brief Private constructor - use probe() to create instances
     * 
     * @details Constructs a MAX7456 backend instance with the provided OSD manager
     *          reference and SPI device handle. The constructor initializes member
     *          variables to default states but does NOT communicate with hardware.
     *          Actual hardware initialization occurs in init().
     *          
     *          Constructor is private to enforce factory pattern via probe() method,
     *          ensuring hardware is properly detected before instance creation.
     * 
     * @param[in] osd Reference to AP_OSD manager instance
     * @param[in] dev SPI device handle with ownership transfer
     * 
     * @note Only called by probe() after successful hardware detection
     * 
     * @see probe()
     */
    AP_OSD_MAX7456(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Add register write command to SPI buffer
     * 
     * @details Appends a register write command to the internal SPI command buffer
     *          for batched transmission. Commands accumulate in the buffer until
     *          sent together in a single SPI transaction, reducing bus overhead.
     *          
     *          Command format: [register_address, value]
     *          
     *          Buffer is flushed when full or during explicit transfer operations.
     * 
     * @param[in] reg MAX7456 register address (VM0, DMM, DMAH, DMAL, etc.)
     * @param[in] arg Value to write to register
     * 
     * @note Buffer overflow is prevented by periodic flushes during frame transfer
     */
    void buffer_add_cmd(uint8_t reg, uint8_t arg);

    /**
     * @brief Update font in MAX7456 NVRAM if changed
     * 
     * @details Checks if the current OSD_FONT parameter differs from the last uploaded
     *          font, and if so, loads the new font data from filesystem and uploads
     *          all 256 characters to MAX7456 NVRAM. Each character is verified after
     *          writing to ensure successful NVRAM programming.
     *          
     *          Font update sequence:
     *          1. Check OSD_FONT parameter vs last_font
     *          2. Load font data from SD card or ROMFS
     *          3. Write each character to NVRAM via CM0-CM5 registers
     *          4. Verify each character using check_font_char()
     *          5. Update last_font on success
     * 
     * @return true if font update successful or not needed
     * @return false if font load or NVRAM write failed
     * 
     * @note Font upload is slow (several hundred ms) and blocks during execution
     * 
     * @see update_font_char()
     * @see check_font_char()
     */
    bool update_font();

    /**
     * @brief Verify font character in MAX7456 NVRAM matches expected data
     * 
     * @details Reads back a character definition from MAX7456 NVRAM and compares
     *          it with the expected font data to verify correct programming.
     *          Character data is 54 bytes (18 rows × 3 bytes/row) in 2-bit format.
     *          
     *          Used during font upload to verify NVRAM writes completed successfully.
     * 
     * @param[in] chr Character code (0-255)
     * @param[in] font_data Pointer to expected 54-byte character bitmap data
     * 
     * @return true if NVRAM content matches expected data
     * @return false if mismatch detected or read failed
     * 
     * @see update_font_char()
     */
    bool check_font_char(uint8_t chr, const uint8_t* font_data);

    /**
     * @brief Write single font character to MAX7456 NVRAM
     * 
     * @details Programs one character definition into MAX7456 character memory (NVRAM).
     *          Character data is written to CM0-CM5 registers (54 bytes total) then
     *          committed to NVRAM using the CMM register write operation.
     *          
     *          MAX7456 character format:
     *          - 18 rows per character
     *          - 3 bytes per row (12 pixels × 2 bits/pixel)
     *          - 2-bit pixels: 00=transparent, 01=black, 10=white, 11=gray
     * 
     * @param[in] chr Character code (0-255) to program
     * @param[in] font_data Pointer to 54-byte character bitmap data
     * 
     * @return true if character write successful
     * @return false if write failed or verification failed
     * 
     * @note NVRAM write requires several milliseconds for internal flash programming
     * 
     * @see check_font_char()
     */
    bool update_font_char(uint8_t chr, const uint8_t* font_data);

    /**
     * @brief Check if reinitialization is needed
     * 
     * @details Monitors video signal status and configuration parameters to determine
     *          if MAX7456 requires reinitialization. Triggers reinit if:
     *          - Video signal lost then recovered (video_detect_time debounce)
     *          - Video format changed (PAL ↔ NTSC switch)
     *          - Horizontal or vertical offset parameters changed
     *          - Font parameter changed
     *          
     *          Signal detection is debounced using VIDEO_SIGNAL_DEBOUNCE_MS timing
     *          to avoid excessive reinitializations on momentary signal glitches.
     * 
     * @note Called periodically from flush() to monitor hardware status
     * 
     * @see reinit()
     */
    void check_reinit();

    /**
     * @brief Reinitialize MAX7456 hardware
     * 
     * @details Performs soft reset and reinitialization of MAX7456 without full
     *          probe sequence. Used when video signal recovers or configuration
     *          parameters change. Reinitializes:
     *          - Video format detection (PAL/NTSC)
     *          - Display row count based on format
     *          - Horizontal and vertical offset registers
     *          - Font update if font parameter changed
     *          - Display mode and enable state
     * 
     * @note Does NOT re-probe hardware or reallocate resources, only reconfigures
     *       existing MAX7456 instance.
     * 
     * @see check_reinit()
     */
    void reinit();

    /**
     * @brief Transfer frame buffer differences to MAX7456 display memory
     * 
     * @details Implements the core differential frame transfer algorithm. Scans through
     *          all character positions, identifies changed positions by comparing frame[]
     *          with shadow_frame[], builds optimized SPI command sequences using DMM
     *          (Display Memory Mode) auto-increment, and transfers commands to hardware.
     *          
     *          Optimization: Uses MAX7456 DMM auto-increment addressing to efficiently
     *          write consecutive changed characters without redundant address commands.
     *          
     *          After transfer, shadow_frame is updated to match current frame state.
     * 
     * @note Called by flush() with SPI semaphore already held
     * 
     * @see flush()
     * @see is_dirty()
     */
    void transfer_frame();

    /**
     * @brief Check if character position has changed since last transfer
     * 
     * @details Compares character at specified position in current frame buffer
     *          with shadow frame buffer to determine if position needs updating.
     *          Used by transfer_frame() to build differential update list.
     * 
     * @param[in] x Column position (0-29)
     * @param[in] y Row position (0-12 NTSC or 0-15 PAL)
     * 
     * @return true if frame[y][x] != shadow_frame[y][x] (character changed)
     * @return false if position unchanged
     * 
     * @see transfer_frame()
     */
    bool is_dirty(uint8_t x, uint8_t y);

    // SPI device handle with exclusive ownership
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    // Last read video signal register value (STAT register, bit 0 = PAL/NTSC)
    uint8_t  video_signal_reg;
    
    // Initialization complete flag (true after successful init())
    bool initialized;
    
    // Last uploaded font index (OSD_FONT parameter value) for change detection
    uint8_t last_font;
    
    // Last configured vertical offset (OSD_V_OFFSET parameter) for change detection
    int8_t last_v_offset;
    
    // Last configured horizontal offset (OSD_H_OFFSET parameter) for change detection
    int8_t last_h_offset;

    /**
     * @brief Number of visible character rows for NTSC video format
     * 
     * @note NTSC (525 lines @ 60Hz) provides 13 rows of 18-pixel tall characters
     *       in the active display area. Row coordinates range from 0-12.
     */
    static const uint8_t video_lines_ntsc = 13;
    
    /**
     * @brief Number of visible character rows for PAL video format
     * 
     * @note PAL (625 lines @ 50Hz) provides 16 rows of 18-pixel tall characters
     *       in the active display area. Row coordinates range from 0-15.
     */
    static const uint8_t video_lines_pal = 16;
    
    /**
     * @brief Number of character columns (same for NTSC and PAL)
     * 
     * @note Both video formats support 30 columns of 12-pixel wide characters.
     *       Column coordinates range from 0-29.
     */
    static const uint8_t video_columns = 30;
    
    /**
     * @brief Size of SPI command buffer in bytes
     * 
     * @note Buffer sized to hold worst-case full frame update commands.
     *       Each character position requires 3 bytes: register address,
     *       high byte, low byte for DMM addressing + character code.
     */
    static const uint16_t spi_buffer_size = 512;

    /**
     * @brief Current frame buffer containing characters to display
     * 
     * @details 2D array storing character codes for each screen position.
     *          Array is sized for maximum PAL resolution (16 rows × 30 columns).
     *          For NTSC, only first 13 rows are used. Character codes (0-255)
     *          reference the uploaded font set in MAX7456 NVRAM.
     *          
     *          Modified by write() and clear(), transferred to hardware by flush().
     */
    uint8_t frame[video_lines_pal][video_columns];

    /**
     * @brief Shadow frame buffer tracking last state transferred to hardware
     * 
     * @details Frame already transferred to MAX7456 display memory, used to optimize
     *          SPI traffic by enabling differential updates. During flush(), only
     *          character positions where frame[y][x] != shadow_frame[y][x] are
     *          sent to hardware, dramatically reducing bus usage for static or
     *          partially-changing displays.
     *          
     *          Updated by transfer_frame() after successful SPI transfer.
     */
    uint8_t shadow_frame[video_lines_pal][video_columns];

    /**
     * @brief SPI command buffer for batched register writes
     * 
     * @details Accumulates multiple register write commands into a single buffer
     *          for transmission in one SPI transaction, reducing bus overhead and
     *          improving efficiency. Commands are 2-byte pairs: [register, value].
     */
    uint8_t buffer[spi_buffer_size];
    
    /**
     * @brief Current write offset in SPI command buffer (bytes used)
     * 
     * @details Tracks number of bytes written to buffer[]. Buffer is flushed
     *          (transmitted and reset) when buffer_offset approaches spi_buffer_size.
     */
    int buffer_offset;

    /**
     * @brief Timestamp (milliseconds) of last video signal status check
     * 
     * @details Used to throttle signal detection checks. Video signal register
     *          is read periodically (not every frame) to reduce SPI overhead.
     */
    uint32_t last_signal_check;
    
    /**
     * @brief Timestamp (milliseconds) when video signal was first detected as stable
     * 
     * @details Used for video signal debouncing. Video format detection requires
     *          signal to remain stable for VIDEO_SIGNAL_DEBOUNCE_MS before accepting
     *          format detection to avoid false triggers from signal glitches.
     */
    uint32_t video_detect_time;

    /**
     * @brief Current number of visible rows based on detected video format
     * 
     * @details Set to video_lines_ntsc (13) or video_lines_pal (16) after video
     *          format detection during init() or reinit(). Used to determine valid
     *          row range for rendering and clipping operations.
     */
    uint16_t video_lines;
};
#endif // HAL_WITH_OSD_BITMAP
