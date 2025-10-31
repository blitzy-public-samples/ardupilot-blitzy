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
 * @file AP_OSD_SITL.h
 * @brief SITL (Software In The Loop) OSD backend for simulation visualization
 * 
 * @details This file implements a graphical On-Screen Display backend for
 *          SITL simulation using the SFML (Simple and Fast Multimedia Library)
 *          graphics library. It provides real-time OSD visualization in a
 *          dedicated window during software simulation, allowing developers
 *          to test and debug OSD layouts without physical hardware.
 *          
 *          The implementation renders character-based OSD overlays similar to
 *          MAX7456-style hardware OSD chips, converting text framebuffer data
 *          to graphical display using bitmap fonts loaded from ROMFS.
 *          
 *          Key Features:
 *          - SFML-based window rendering for cross-platform simulation
 *          - MAX7456-compatible character grid layout (30 columns × variable rows)
 *          - Font texture loading from ROMFS (fontN.bin format)
 *          - 2-bit glyph decoding to RGBA texture conversion
 *          - Threaded rendering with mutex synchronization
 *          - Configurable character scaling for improved readability
 * 
 * @note This backend is only available in SITL builds with WITH_SITL_OSD defined
 * @warning Requires SFML graphics library for compilation and runtime
 * 
 * @see AP_OSD_Backend for the base OSD backend interface
 * @see AP_OSD for the main OSD management class
 */
#pragma once

#ifdef WITH_SITL_OSD

#include <AP_OSD/AP_OSD_Backend.h>

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif

/**
 * @class AP_OSD_SITL
 * @brief SFML-based graphical OSD renderer for SITL simulation
 * 
 * @details AP_OSD_SITL provides a visual representation of the on-screen display
 *          during software-in-the-loop simulation. It creates an SFML window that
 *          renders character-based OSD data as graphical output, mimicking the
 *          behavior of hardware OSD chips like the MAX7456.
 *          
 *          Architecture:
 *          - Maintains a character framebuffer matching MAX7456 layout (30 columns)
 *          - Loads bitmap fonts from ROMFS (fontN.bin files with 2-bit glyph data)
 *          - Converts 2-bit glyphs to RGBA textures for SFML rendering
 *          - Runs SFML rendering loop in separate POSIX thread for responsiveness
 *          - Uses HAL_Semaphore (mutex) for thread-safe framebuffer access
 *          
 *          Coordinate System:
 *          - Character positions: x (column 0-29), y (row 0-15 typical)
 *          - Pixel coordinates: character_pos * char_width * char_scale
 *          - Origin (0,0) at top-left corner of display
 *          
 *          Font Format:
 *          - Each font file contains 256 character glyphs
 *          - Glyph size: 12×18 pixels (char_width × char_height)
 *          - Encoding: 2 bits per pixel (4 levels: transparent, black, white, gray)
 *          - Converted to RGBA textures with appropriate alpha blending
 *          
 *          Threading Model:
 *          - Main thread: write(), clear(), flush() called by OSD update loop
 *          - Render thread: SFML window event handling and display updates
 *          - Synchronization: AP::notify().sf_window_mutex protects SFML window access
 *          - Framebuffer: Protected by internal mutex for thread-safe access
 *          
 *          Display Scaling:
 *          - Base character size: 12×18 pixels (MAX7456 compatible)
 *          - Scaling factor: 2× (char_scale) for improved readability
 *          - Final rendered size: 24×36 pixels per character
 *          - Window size: (video_cols × 24) × (video_lines × 36) pixels
 * 
 * @note This class is only instantiated in SITL builds with WITH_SITL_OSD
 * @warning Thread synchronization required: SFML window operations must be
 *          synchronized using AP::notify().sf_window_mutex to prevent race
 *          conditions between rendering thread and SFML event loop
 * @warning SFML library must be initialized before probe() is called
 * 
 * @see AP_OSD_Backend for base class interface requirements
 * @see AP_OSD::osd_types::OSD_SITL for backend type enumeration
 */
class AP_OSD_SITL : public AP_OSD_Backend
{

public:
    /**
     * @brief Probe and initialize SITL OSD backend
     * 
     * @details Factory method that creates and initializes an AP_OSD_SITL instance
     *          for SITL simulation. This method constructs the backend object and
     *          calls init() to set up the SFML window, load fonts from ROMFS, and
     *          start the rendering thread.
     *          
     *          Initialization sequence:
     *          1. Construct AP_OSD_SITL instance
     *          2. Call init() to set up SFML window
     *          3. Load font textures from ROMFS (fontN.bin)
     *          4. Allocate framebuffer memory
     *          5. Start POSIX rendering thread
     *          6. Return backend pointer on success, nullptr on failure
     * 
     * @param[in] osd Reference to parent AP_OSD instance for configuration access
     * 
     * @return Pointer to initialized AP_OSD_SITL backend on success, nullptr on failure
     * 
     * @note Probe is called during OSD subsystem initialization in SITL builds
     * @note Failure can occur if SFML window creation fails or fonts cannot be loaded
     * 
     * @see init() for detailed initialization steps
     * @see AP_OSD_Backend::probe() for base class documentation
     */
    static AP_OSD_Backend *probe(AP_OSD &osd);

    /**
     * @brief Write text string to framebuffer at specified character position
     * 
     * @details Draws the given text string into the character framebuffer at the
     *          specified column (x) and row (y) position. The text is copied into
     *          the internal buffer and will be rendered to the SFML window when
     *          flush() is called.
     *          
     *          Character positioning uses a grid layout matching MAX7456 hardware:
     *          - Columns: 0 to (video_cols-1), typically 0-29
     *          - Rows: 0 to (video_lines-1), typically 0-15
     *          
     *          Each character in the text string is written sequentially from left
     *          to right starting at position (x,y). Text that extends beyond the
     *          right edge of the display is clipped.
     *          
     *          Thread Safety: This method accesses the framebuffer and should be
     *          called from the main OSD update thread. The internal mutex protects
     *          concurrent access during rendering.
     * 
     * @param[in] x Column position (0 to video_cols-1, typically 0-29)
     * @param[in] y Row position (0 to video_lines-1, typically 0-15)
     * @param[in] text Null-terminated ASCII string to display
     * 
     * @note Text characters are ASCII codes used as indices into font texture array
     * @note Characters outside valid x,y range are silently ignored (clipped)
     * @note This method only updates the framebuffer; call flush() to render to screen
     * 
     * @see flush() to render framebuffer contents to SFML window
     * @see clear() to reset framebuffer before writing new content
     */
    void write(uint8_t x, uint8_t y, const char* text) override;

    /**
     * @brief Initialize SFML display window and rendering subsystem
     * 
     * @details Initializes the SITL OSD backend by setting up the SFML graphics
     *          environment, loading font textures from ROMFS, allocating the
     *          framebuffer, and starting the rendering thread.
     *          
     *          Initialization Steps:
     *          1. Calculate display dimensions based on video standard (PAL/NTSC)
     *          2. Allocate character framebuffer (video_cols × video_lines bytes)
     *          3. Create SFML RenderWindow with appropriate size
     *          4. Load font textures from ROMFS files (font0.bin, font1.bin, etc.)
     *          5. Convert 2-bit glyph data to RGBA textures for SFML rendering
     *          6. Create POSIX thread for SFML rendering loop (update_thread)
     *          7. Initialize mutex for thread-safe framebuffer access
     *          
     *          Font Loading:
     *          - Fonts stored in ROMFS as fontN.bin files (N = font index 0-255)
     *          - Each font file contains 256 character glyphs
     *          - Glyph format: 12×18 pixels at 2 bits per pixel
     *          - 2-bit encoding: 0=transparent, 1=black, 2=white, 3=gray
     *          - Converted to RGBA sf::Texture for hardware-accelerated rendering
     *          
     *          Window Configuration:
     *          - Title: "ArduPilot SITL OSD"
     *          - Size: (video_cols × char_width × char_scale) × (video_lines × char_height × char_scale)
     *          - Default: 720×576 pixels (30 cols × 16 rows × 2× scale)
     *          - Resizable: No (fixed aspect ratio)
     * 
     * @return true if initialization successful (window created, fonts loaded, thread started)
     * @return false if initialization failed (SFML error, font load error, thread creation failed)
     * 
     * @note Called once during backend probe() or OSD subsystem initialization
     * @note Failure typically indicates missing SFML library or corrupt ROMFS fonts
     * @warning Must be called before any write(), clear(), or flush() operations
     * @warning SFML window operations require proper thread synchronization via AP::notify().sf_window_mutex
     * 
     * @see load_font() for font texture loading details
     * @see update_thread() for rendering loop implementation
     * @see AP_OSD_Backend::init() for base class interface
     */
    bool init() override;

    /**
     * @brief Trigger rendering of framebuffer contents to SFML window
     * 
     * @details Signals the rendering thread to update the SFML window with current
     *          framebuffer contents. This method increments a counter that the
     *          rendering thread monitors to detect when new frame data is available.
     *          
     *          Rendering Flow:
     *          1. Main thread calls write() multiple times to populate framebuffer
     *          2. Main thread calls flush() to signal frame complete
     *          3. Flush increments counter variable
     *          4. Rendering thread (update_thread) detects counter change
     *          5. Rendering thread locks mutex and reads framebuffer
     *          6. Rendering thread renders characters to SFML window using font textures
     *          7. Rendering thread calls window.display() to present frame
     *          
     *          Frame Buffer to Display Mapping:
     *          - Each character in buffer[y][x] mapped to screen position (x×24, y×36) pixels
     *          - Character value used as index into font[] texture array
     *          - Texture rendered as sprite at scaled position with 2× scaling factor
     *          - Aspect ratio preserved: 12×18 base × 2 scale = 24×36 pixels per char
     *          
     *          Thread Synchronization:
     *          - Counter variable protected by internal mutex
     *          - Rendering thread uses AP::notify().sf_window_mutex for SFML window access
     *          - Non-blocking: flush() returns immediately, actual rendering asynchronous
     * 
     * @note This method must be called after write() operations to make changes visible
     * @note Flush is non-blocking; rendering occurs asynchronously in separate thread
     * @note Multiple flush() calls before rendering completes are coalesced (only counter increments)
     * 
     * @see write() for populating framebuffer with text
     * @see update_thread() for actual SFML rendering implementation
     * @see clear() to reset framebuffer before next frame
     */
    void flush() override;

    /**
     * @brief Clear framebuffer by filling with spaces (blank characters)
     * 
     * @details Resets the entire character framebuffer to ASCII space characters (0x20),
     *          effectively blanking the display. This method should be called at the
     *          beginning of each OSD frame update cycle before writing new content.
     *          
     *          Clear Operation:
     *          - Fills buffer with 0x20 (space character) for all positions
     *          - Buffer size: video_cols × video_lines bytes
     *          - Typical size: 30 columns × 16 rows = 480 bytes
     *          - Thread-safe: Protected by internal mutex
     *          
     *          Usage Pattern:
     *          1. clear() - Blank the framebuffer
     *          2. write() - Add text elements (altitude, speed, mode, etc.)
     *          3. write() - Add more elements at different positions
     *          4. flush() - Render complete frame to window
     * 
     * @note Clear does not immediately affect the displayed window; call flush() to update display
     * @note Space character (0x20) renders as transparent in most OSD fonts
     * @note Called automatically by OSD update loop before rendering new frame
     * 
     * @see write() for adding text after clearing
     * @see flush() for rendering cleared framebuffer to display
     */
    void clear() override;

    /**
     * @brief Check if this backend can coexist with another backend type
     * 
     * @details Determines whether AP_OSD_SITL can operate simultaneously with
     *          another OSD backend type. SITL OSD can coexist with telemetry-only
     *          backends (OSD_TXONLY, OSD_MSP, OSD_MSP_DISPLAYPORT) but not with
     *          hardware display backends that would conflict for screen output.
     *          
     *          Compatibility Matrix:
     *          - OSD_MAX7456: Incompatible (hardware display conflict)
     *          - OSD_SITL: Incompatible (duplicate SITL backend)
     *          - OSD_NONE: Compatible (no conflict)
     *          - OSD_TXONLY: Compatible (telemetry only, no display)
     *          - OSD_MSP: Compatible (telemetry to external OSD)
     *          - OSD_MSP_DISPLAYPORT: Compatible (telemetry to external display)
     * 
     * @param[in] type Backend type to check compatibility against
     * 
     * @return true if backends can coexist without conflict
     * @return false if backends would conflict (both render to display)
     * 
     * @note Used by AP_OSD to determine valid backend combinations
     * @see AP_OSD_Backend::is_compatible_with_backend_type() for base interface
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
     * @brief Get the backend type identifier for this SITL OSD
     * 
     * @details Returns the OSD backend type enumeration value identifying this
     *          backend as the SITL simulation OSD implementation.
     * 
     * @return AP_OSD::osd_types::OSD_SITL backend type identifier
     * 
     * @note Used by AP_OSD for backend identification and logging
     * @see AP_OSD_Backend::get_backend_type() for base interface
     */
    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_SITL;
    }
private:
    /**
     * @brief Private constructor for SITL OSD backend
     * 
     * @details Constructs an AP_OSD_SITL instance. Constructor is private to enforce
     *          factory pattern; instances must be created through probe() method.
     *          
     *          Initializes member variables and sets up the backend with reference
     *          to parent AP_OSD instance for configuration parameter access.
     * 
     * @param[in] osd Reference to parent AP_OSD instance
     * 
     * @note Constructor does not allocate resources; call init() after construction
     * @see probe() for the public factory method
     */
    AP_OSD_SITL(AP_OSD &osd);

    /**
     * @brief SFML render window pointer for graphical display
     * 
     * @details Pointer to the SFML RenderWindow that displays the OSD overlay.
     *          Created during init() and managed by the rendering thread.
     *          Window size calculated as: (video_cols × char_width × char_scale) ×
     *          (video_lines × char_height × char_scale) pixels.
     * 
     * @note Window operations must be synchronized with AP::notify().sf_window_mutex
     * @warning SFML window is not thread-safe; always lock mutex before access
     */
    sf::RenderWindow *w;

    /**
     * @brief Array of SFML font textures for character rendering
     * 
     * @details Contains 256 sf::Texture objects, one for each possible character
     *          in the font. Each texture is a 12×18 pixel RGBA image decoded from
     *          2-bit glyph data loaded from ROMFS fontN.bin files.
     *          
     *          Texture Format:
     *          - Size: 12×18 pixels (char_width × char_height)
     *          - Format: RGBA (32 bits per pixel)
     *          - Source: 2-bit encoded glyphs (4 levels per pixel)
     *          - Encoding: 0=transparent, 1=black, 2=white, 3=gray
     *          - Alpha: Transparency for overlay blending
     * 
     * @note Textures loaded once during init() and reused for all rendering
     * @note Character code used as index: font[char_code]
     * @see load_font() for texture creation from ROMFS font files
     */
    sf::Texture font[256];
    
    /**
     * @brief Last loaded font index
     * 
     * @details Tracks which font file was most recently loaded (0-255).
     *          Used for font switching if multiple font banks are supported.
     * 
     * @note Currently fonts are loaded once at initialization
     */
    uint8_t last_font;

    /**
     * @brief Character width in pixels (MAX7456 compatible)
     * 
     * @details Base width of each character glyph in pixels, matching MAX7456
     *          hardware OSD chip character dimensions. Actual rendered size is
     *          char_width × char_scale.
     * 
     * @note Value: 12 pixels (standard MAX7456 character width)
     */
    static const uint8_t char_width = 12;
    
    /**
     * @brief Character height in pixels (MAX7456 compatible)
     * 
     * @details Base height of each character glyph in pixels, matching MAX7456
     *          hardware OSD chip character dimensions. Actual rendered size is
     *          char_height × char_scale.
     * 
     * @note Value: 18 pixels (standard MAX7456 character height)
     */
    static const uint8_t char_height = 18;
    
    /**
     * @brief Number of character rows in vertical direction
     * 
     * @details Vertical character count, depends on video standard (PAL/NTSC).
     *          Typical values: 16 rows for PAL, 13 rows for NTSC.
     *          Configured during init() based on video standard setting.
     * 
     * @note Determines framebuffer height: video_lines rows
     */
    uint8_t video_lines;
    
    /**
     * @brief Number of character columns in horizontal direction
     * 
     * @details Horizontal character count, matching MAX7456 standard layout.
     *          Fixed at 30 columns for compatibility with MAX7456 hardware.
     * 
     * @note Value: 30 columns (MAX7456 standard)
     * @note Determines framebuffer width: video_cols columns
     */
    uint8_t video_cols;
    
    /**
     * @brief Spacing between characters in pixels
     * 
     * @details Additional horizontal spacing between adjacent characters.
     *          Set to 0 for continuous character placement without gaps.
     * 
     * @note Value: 0 pixels (no spacing, characters placed adjacently)
     */
    static const uint8_t char_spacing = 0;

    /**
     * @brief Display scaling factor for improved readability
     * 
     * @details Multiplication factor applied to character dimensions for rendering.
     *          Characters are rendered at (char_width × char_scale) × (char_height × char_scale)
     *          pixels to improve visibility on modern high-resolution displays.
     *          
     *          Effective size: 12×2 = 24 pixels wide, 18×2 = 36 pixels tall per character.
     * 
     * @note Value: 2× scaling (24×36 pixels per character)
     * @note Higher values increase window size and improve readability
     */
    static const uint8_t char_scale = 2;

    /**
     * @brief Helper function to access framebuffer at character position
     * 
     * @details Calculates linear buffer offset from 2D character coordinates (x, y)
     *          and returns reference to the character byte at that position.
     *          
     *          Buffer layout: Row-major order (rows stored sequentially)
     *          Offset calculation: y × video_cols + x
     * 
     * @param[in] buf Pointer to character framebuffer
     * @param[in] y Row position (0 to video_lines-1)
     * @param[in] x Column position (0 to video_cols-1)
     * 
     * @return Reference to character byte at position (x, y) in buffer
     * 
     * @note Inline helper for efficient buffer access without bounds checking
     * @warning No bounds checking; caller must ensure x, y are within valid range
     */
    uint8_t &getbuffer(uint8_t *buf, uint8_t y, uint8_t x) const {
        return buf[y*uint32_t(video_cols) + x];
    }

    /**
     * @brief Character framebuffer storing current display contents
     * 
     * @details Dynamic array storing ASCII character codes for each position
     *          on the OSD display. Size: video_cols × video_lines bytes.
     *          Typical size: 30 × 16 = 480 bytes.
     *          
     *          Buffer organization:
     *          - Row-major order: [row0_col0, row0_col1, ..., row0_col29, row1_col0, ...]
     *          - Each byte: ASCII character code (0-255)
     *          - Value used as index into font[] texture array for rendering
     *          
     *          Thread access:
     *          - Main thread: write() and clear() update buffer
     *          - Render thread: update_thread() reads buffer for display
     *          - Protection: mutex synchronizes access between threads
     * 
     * @note Allocated during init(), deallocated in destructor
     * @warning Must be accessed with mutex locked for thread safety
     */
    uint8_t *buffer;

    /**
     * @brief Rendering thread main loop for SFML window updates
     * 
     * @details Main loop running in separate POSIX thread that handles SFML
     *          window events and renders framebuffer contents to display.
     *          
     *          Rendering Loop:
     *          1. Wait for counter change (flush() called)
     *          2. Lock AP::notify().sf_window_mutex for SFML window access
     *          3. Process SFML window events (close, resize, focus)
     *          4. Clear window with background color
     *          5. Lock internal mutex and read framebuffer
     *          6. For each character position (y, x):
     *             - Get character code from buffer[y][x]
     *             - Create sprite from font[char_code] texture
     *             - Position sprite at (x × char_width × char_scale, y × char_height × char_scale)
     *             - Scale sprite by char_scale factor
     *             - Draw sprite to window
     *          7. Display window (present frame)
     *          8. Unlock mutexes and repeat
     *          
     *          Frame synchronization:
     *          - Monitors counter variable for changes
     *          - Renders new frame when counter != last_counter
     *          - Updates last_counter after rendering
     * 
     * @note Runs in separate pthread created during init()
     * @note Window event handling prevents freezing and allows window close
     * @warning Must use AP::notify().sf_window_mutex for all SFML window operations
     * @warning Must lock internal mutex when reading framebuffer
     * 
     * @see update_thread_start() for pthread entry point
     * @see flush() for counter increment that triggers rendering
     */
    void update_thread();
    
    /**
     * @brief Static pthread entry point for rendering thread
     * 
     * @details Static function that serves as the entry point for the POSIX
     *          rendering thread. Casts the void* argument to AP_OSD_SITL* and
     *          calls the instance method update_thread().
     *          
     *          Pthread Interface:
     *          - Signature: void* (*)(void*) required by pthread_create()
     *          - Parameter: void* obj is actually AP_OSD_SITL* this pointer
     *          - Return: nullptr when thread terminates
     * 
     * @param[in] obj Pointer to AP_OSD_SITL instance (cast from void*)
     * 
     * @return nullptr on thread exit
     * 
     * @note Called automatically by pthread library, not by user code
     * @see update_thread() for actual rendering loop implementation
     */
    static void *update_thread_start(void *obj);
    
    /**
     * @brief Load font textures from ROMFS font files
     * 
     * @details Loads bitmap font data from ROMFS fontN.bin files and converts
     *          2-bit glyph data to SFML RGBA textures for rendering.
     *          
     *          Font File Format:
     *          - Location: ROMFS filesystem (compiled into binary)
     *          - Filename: fontN.bin (N = font index, typically 0)
     *          - Content: 256 character glyphs
     *          - Glyph size: 12×18 pixels
     *          - Encoding: 2 bits per pixel (4 gray levels)
     *          - Total size: 256 chars × 12 pixels × 18 pixels × 2 bits/pixel ÷ 8 = 13,824 bytes
     *          
     *          2-bit to RGBA Conversion:
     *          - 2-bit value 0 (00): Transparent (RGBA: 0, 0, 0, 0)
     *          - 2-bit value 1 (01): Black (RGBA: 0, 0, 0, 255)
     *          - 2-bit value 2 (10): White (RGBA: 255, 255, 255, 255)
     *          - 2-bit value 3 (11): Gray (RGBA: 128, 128, 128, 255)
     *          
     *          Loading Process:
     *          1. Open fontN.bin from ROMFS
     *          2. Read 2-bit glyph data for all 256 characters
     *          3. For each glyph:
     *             - Decode 2-bit pixels to RGBA values
     *             - Create 12×18 RGBA image
     *             - Upload to sf::Texture for GPU rendering
     *          4. Store texture in font[] array
     * 
     * @note Called during init() before starting rendering thread
     * @note Font textures remain loaded for entire backend lifetime
     * @warning Initialization fails if font file cannot be loaded from ROMFS
     * 
     * @see init() for initialization sequence
     */
    void load_font();

    /**
     * @brief POSIX thread handle for rendering thread
     * 
     * @details Thread handle returned by pthread_create() for the rendering thread.
     *          Used for thread management (join, cancel) during shutdown.
     * 
     * @note Thread created in init(), joined in destructor
     * @see update_thread_start() for thread entry point
     */
    pthread_t thread;
    
    /**
     * @brief HAL semaphore for framebuffer thread synchronization
     * 
     * @details Mutex protecting framebuffer access between main thread (write/clear)
     *          and rendering thread (update_thread). Ensures thread-safe read/write
     *          operations on the buffer without data races.
     *          
     *          Protected Operations:
     *          - write(): Locks mutex while updating buffer contents
     *          - clear(): Locks mutex while clearing buffer
     *          - update_thread(): Locks mutex while reading buffer for rendering
     * 
     * @note HAL_Semaphore provides RTOS-compatible mutex abstraction
     * @warning Always lock before accessing buffer to prevent race conditions
     */
    HAL_Semaphore mutex;
    
    /**
     * @brief Frame counter for flush synchronization
     * 
     * @details Incremented by flush() to signal that new framebuffer data is
     *          available for rendering. Rendering thread monitors this counter
     *          and renders a new frame when counter changes.
     *          
     *          Synchronization Flow:
     *          1. Main thread calls write() to update framebuffer
     *          2. Main thread calls flush(), which increments counter
     *          3. Rendering thread detects counter != last_counter
     *          4. Rendering thread renders frame and updates last_counter
     * 
     * @note Protected by mutex for thread-safe access
     * @see flush() for counter increment
     * @see last_counter for previous counter value tracking
     */
    uint32_t counter;
    
    /**
     * @brief Previous frame counter value for change detection
     * 
     * @details Stores the counter value from the previous rendering cycle.
     *          Rendering thread compares counter to last_counter to detect
     *          when new frame data is available.
     *          
     *          When counter != last_counter:
     *          - New frame data available
     *          - Render framebuffer to window
     *          - Update last_counter = counter
     * 
     * @note Updated by rendering thread after each frame render
     * @see counter for current frame counter value
     * @see update_thread() for change detection logic
     */
    uint32_t last_counter;
};

#endif // WITH_SITL_OSD
