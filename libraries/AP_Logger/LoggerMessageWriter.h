/**
 * @file LoggerMessageWriter.h
 * @brief Non-blocking writers for log startup metadata messages
 * 
 * @details This file implements a family of writer classes that emit
 *          critical startup metadata messages to DataFlash logs without
 *          blocking the main logging system. These writers handle:
 *          - FMT (format definitions for all log message types)
 *          - UNIT (unit definitions for log fields)
 *          - MULT (multiplier definitions for fixed-point scaling)
 *          - PARM (all vehicle parameters)
 *          - VER (version information)
 *          - Mission items, rally points, and geofence data
 *          - Vehicle-specific startup messages
 * 
 *          The non-blocking design uses a stage-based state machine pattern
 *          where each call to process() writes a small amount of data and
 *          advances to the next stage, preventing logging initialization
 *          from blocking time-critical flight code.
 * 
 * @note These writers are instantiated during AP_Logger::Init() and run
 *       during the logging startup sequence.
 * 
 * @warning Writers execute in the logging thread context. Careful timing
 *          management is required as startup message emission can take
 *          hundreds of milliseconds for large parameter sets or missions.
 * 
 * @see AP_Logger::Init()
 * @see AP_Logger_Backend::_startup_messagewriter
 */

#pragma once

#include "AP_Logger_Backend.h"
#include <AP_Rally/AP_Rally.h>

/**
 * @class LoggerMessageWriter
 * @brief Abstract base class for non-blocking log message writers
 * 
 * @details This base class defines the interface for all startup message
 *          writers that emit metadata to DataFlash logs. Each writer
 *          implements a stage-based state machine that progresses through
 *          multiple process() calls without blocking.
 * 
 *          The non-blocking pattern is critical because:
 *          - Logging initialization occurs during vehicle startup
 *          - Large parameter sets (200+ params) take significant time to write
 *          - Mission/rally/fence data can be substantial
 *          - Flight-critical code cannot be blocked during this process
 * 
 *          Derived classes implement specific metadata writing tasks:
 *          - System information and version data
 *          - Mission waypoints
 *          - Rally points
 *          - Geofence polygons
 *          - Complete startup sequence orchestration
 * 
 * @note Writers maintain internal state across process() calls and must
 *       be reset() before reuse.
 * 
 * @warning Writers are not thread-safe. They are designed to be called
 *          from the logging thread only.
 */
class LoggerMessageWriter {
public:

    /**
     * @brief Reset the writer to initial state for reuse
     * 
     * @details Clears internal state and resets stage progression.
     *          Must be called before reusing a writer instance.
     */
    virtual void reset() = 0;

    /**
     * @brief Process one stage of message writing (non-blocking)
     * 
     * @details Writes a small batch of messages and advances internal
     *          state. Call repeatedly until finished() returns true.
     *          Each call is designed to complete quickly (typically <10ms)
     *          to avoid blocking logging operations.
     * 
     * @note Implementation must check out_of_time_for_writing_messages()
     *       to yield control if time budget is exceeded.
     */
    virtual void process() = 0;

    /**
     * @brief Check if writer has completed all stages
     * 
     * @return true if all messages have been written, false if more process() calls needed
     */
    bool finished() const { return _finished; }

    /**
     * @brief Associate this writer with a logger backend
     * 
     * @param[in] backend Pointer to the logger backend that will receive messages
     * 
     * @note Must be called before process() to establish write destination
     */
    virtual void set_logger_backend(class AP_Logger_Backend *backend) {
        _logger_backend = backend;
    }

    /**
     * @brief Check if time budget for writing has been exceeded
     * 
     * @return true if writer should yield control, false if more writing time available
     * 
     * @details Prevents writers from blocking logging thread for too long.
     *          Implementations should check this periodically within process().
     */
    bool out_of_time_for_writing_messages() const;

protected:
    bool _finished = false;  ///< True when writer has completed all stages
    AP_Logger_Backend *_logger_backend = nullptr;  ///< Backend to write messages to
};


/**
 * @class LoggerMessageWriter_WriteSysInfo
 * @brief Writer for system information and version metadata messages
 * 
 * @details Emits critical system identification and configuration messages
 *          to the log during startup. These messages enable log analysis
 *          tools to identify the vehicle configuration and firmware version.
 * 
 *          Messages written:
 *          - Firmware version string (ArduCopter, ArduPlane, etc.)
 *          - Git hash and version information
 *          - VER message (structured version data)
 *          - System ID (MAVLink system/component IDs)
 *          - Parameter storage utilization
 *          - RC protocol in use
 *          - RC output configuration
 * 
 * @note This writer completes quickly (typically <50ms) as it writes
 *       only a small number of fixed messages.
 */
class LoggerMessageWriter_WriteSysInfo : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    /**
     * @brief Stages for writing system information messages
     * 
     * Stage progression:
     * - FIRMWARE_STRING: Write firmware type and version string
     * - GIT_VERSIONS: Write git commit hash and branch information
     * - VER: Write structured VER message with version components
     * - SYSTEM_ID: Write MAVLink system and component IDs
     * - PARAM_SPACE_USED: Write parameter storage utilization stats
     * - RC_PROTOCOL: Write detected RC input protocol (SBUS, PPM, etc.)
     * - RC_OUTPUT: Write RC output channel configuration
     */
    enum class Stage : uint8_t {
        FIRMWARE_STRING = 0,
        GIT_VERSIONS,
        VER,  // i.e. the "VER" message
        SYSTEM_ID,
        PARAM_SPACE_USED,
        RC_PROTOCOL,
        RC_OUTPUT,
    };
    Stage stage;
};

/**
 * @class LoggerMessageWriter_WriteEntireMission
 * @brief Writer for mission waypoint data to log
 * 
 * @details Writes the complete mission plan to the log during startup,
 *          enabling log analysis to correlate vehicle behavior with
 *          planned mission waypoints. Particularly useful for analyzing
 *          AUTO mode flights and mission execution.
 * 
 *          The writer emits:
 *          - Mission metadata (total count, current waypoint)
 *          - All mission items with full MAVLink command parameters
 * 
 * @note Large missions (200+ waypoints) can take significant time to write.
 *       Writer yields periodically to avoid blocking logging thread.
 * 
 * @warning Mission items are written one per process() call to maintain
 *          non-blocking behavior. Very large missions may take several
 *          seconds to fully log.
 */
class LoggerMessageWriter_WriteEntireMission : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    /**
     * @brief Stages for writing mission data
     * 
     * Stage progression:
     * - WRITE_NEW_MISSION_MESSAGE: Write mission metadata (count, active waypoint)
     * - WRITE_MISSION_ITEMS: Iterate through all mission items, writing one per call
     * - DONE: Mission writing complete
     */
    enum class Stage {
        WRITE_NEW_MISSION_MESSAGE = 0,
        WRITE_MISSION_ITEMS,
        DONE
    };

    uint16_t _mission_number_to_send;  ///< Index of next mission item to write
    Stage stage;
};

/**
 * @class LoggerMessageWriter_WriteAllRallyPoints
 * @brief Writer for rally point locations to log
 * 
 * @details Writes all configured rally points to the log during startup.
 *          Rally points are alternate landing/RTL locations used when
 *          the home position is unsuitable or when rally points provide
 *          safer landing options.
 * 
 *          The writer emits:
 *          - Rally point metadata (total count)
 *          - All rally point locations with altitude and flags
 * 
 * @note Rally point count is typically small (<10 points) so this
 *       writer completes quickly.
 * 
 * @see AP_Rally for rally point management
 */
class LoggerMessageWriter_WriteAllRallyPoints : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    /**
     * @brief Stages for writing rally point data
     * 
     * Stage progression:
     * - WRITE_NEW_RALLY_MESSAGE: Write rally metadata (total point count)
     * - WRITE_ALL_RALLY_POINTS: Iterate through all rally points, writing one per call
     * - DONE: Rally point writing complete
     */
    enum class Stage {
        WRITE_NEW_RALLY_MESSAGE = 0,
        WRITE_ALL_RALLY_POINTS,
        DONE
    };

    uint16_t _rally_number_to_send;  ///< Index of next rally point to write
    Stage stage = Stage::WRITE_NEW_RALLY_MESSAGE;
};

#if HAL_LOGGER_FENCE_ENABLED
/**
 * @class LoggerMessageWriter_Write_Polyfence
 * @brief Writer for geofence polygon data to log
 * 
 * @details Writes all configured geofence boundaries to the log during
 *          startup. Geofences define inclusion and exclusion zones that
 *          trigger failsafe actions when breached. Logging fence data
 *          enables post-flight analysis of fence breaches and vehicle
 *          behavior relative to boundaries.
 * 
 *          The writer emits:
 *          - Fence metadata (total point count, fence configuration)
 *          - All fence polygon vertices
 *          - Fence type information (inclusion/exclusion, altitude limits)
 * 
 * @note Complex polygon fences with hundreds of vertices can take
 *       significant time to write. Writer yields periodically.
 * 
 * @warning Only compiled when HAL_LOGGER_FENCE_ENABLED is defined.
 * 
 * @see AC_Fence for geofencing implementation
 */
class LoggerMessageWriter_Write_Polyfence : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    /**
     * @brief Stages for writing geofence data
     * 
     * Stage progression:
     * - WRITE_NEW_FENCE_MESSAGE: Write fence metadata (point count, config)
     * - WRITE_FENCE_ITEMS: Iterate through all fence points, writing one per call
     * - DONE: Fence writing complete
     */
    enum class Stage {
        WRITE_NEW_FENCE_MESSAGE = 0,
        WRITE_FENCE_ITEMS,
        DONE
    };

    uint16_t _fence_number_to_send;  ///< Index of next fence point to write
    Stage stage;
};
#endif // HAL_LOGGER_FENCE_ENABLED

/**
 * @class LoggerMessageWriter_DFLogStart
 * @brief Orchestrator for complete log startup message sequence
 * 
 * @details This is the main coordinator class that manages the entire
 *          startup message writing sequence for DataFlash logs. It ensures
 *          all critical metadata is written in the correct order:
 * 
 *          1. FMT messages - Format definitions for all log message types
 *          2. UNIT messages - Unit definitions (meters, degrees, etc.)
 *          3. MULT messages - Multiplier definitions for fixed-point scaling
 *          4. PARM messages - All vehicle parameters (can be 200+ messages)
 *          5. Vehicle-specific messages - Version info, system config
 *          6. Mission/Rally/Fence data - Navigation and safety boundaries
 * 
 *          The orchestrator uses sub-writers for different data types and
 *          manages time budgeting to prevent blocking the logging thread
 *          for excessive periods.
 * 
 * @note This writer can take several seconds to complete for vehicles with:
 *       - Large parameter sets (200+ parameters)
 *       - Complex missions (100+ waypoints)
 *       - Detailed geofences (100+ vertices)
 * 
 * @warning Memory allocation occurs during initialization. All sub-writers
 *          must be successfully allocated before use.
 * 
 * @warning Time-critical code must not wait for this writer to finish.
 *          The non-blocking design ensures flight control is not disrupted.
 * 
 * @see AP_Logger::Init() where this writer is instantiated and started
 * @see AP_Logger_Backend::_startup_messagewriter
 */
class LoggerMessageWriter_DFLogStart : public LoggerMessageWriter {
public:
    LoggerMessageWriter_DFLogStart() :
        _writesysinfo()
#if AP_MISSION_ENABLED
        , _writeentiremission()
#endif
#if HAL_RALLY_ENABLED
        , _writeallrallypoints()
#endif
#if HAL_LOGGER_FENCE_ENABLED
        , _writeallpolyfence()
#endif
        {
        }

    /**
     * @brief Associate this writer and all sub-writers with a logger backend
     * 
     * @param[in] backend Pointer to the logger backend that will receive all messages
     * 
     * @note Sets backend for this writer and all owned sub-writers (sysinfo,
     *       mission, rally, fence writers).
     */
    void set_logger_backend(class AP_Logger_Backend *backend) override final {
        LoggerMessageWriter::set_logger_backend(backend);
        _writesysinfo.set_logger_backend(backend);
#if AP_MISSION_ENABLED
        _writeentiremission.set_logger_backend(backend);
#endif
#if HAL_RALLY_ENABLED
        _writeallrallypoints.set_logger_backend(backend);
#endif
#if HAL_LOGGER_FENCE_ENABLED
        _writeallpolyfence.set_logger_backend(backend);
#endif
    }

    /**
     * @brief Check if time budget for DataFlash writing has been exceeded
     * 
     * @return true if writer should yield, false if more time available
     * 
     * @details Used internally to enforce time limits on startup message
     *          writing operations.
     */
    bool out_of_time_for_writing_messages_df() const;

    void reset() override;
    void process() override;

    /**
     * @brief Check if format (FMT) messages have been written
     * 
     * @return true if FMT stage complete, false otherwise
     * 
     * @note FMT messages must be written before any data messages or
     *       log analysis tools cannot decode the log.
     */
    bool fmt_done() const { return _fmt_done; }

    /**
     * @brief Check if parameter (PARM) messages have been written
     * 
     * @return true if PARM stage complete, false otherwise
     * 
     * @note Parameter writing can take significant time (200+ messages).
     */
    bool params_done() const { return _params_done; }

    /**
     * @brief Request mission data to be written to log again
     * 
     * @return true if mission rewrite initiated, false if writer not in DONE state
     * 
     * @details Allows mission data to be re-logged after changes without
     *          restarting entire logging system. Only works when writer
     *          is in DONE state.
     * 
     * @note Useful when mission is updated mid-flight and needs re-logging.
     */
#if AP_MISSION_ENABLED
    bool writeentiremission();
#endif

    /**
     * @brief Request rally point data to be written to log again
     * 
     * @return true if rally rewrite initiated, false if writer not in DONE state
     * 
     * @details Allows rally point data to be re-logged after changes without
     *          restarting entire logging system. Only works when writer
     *          is in DONE state.
     */
#if HAL_RALLY_ENABLED
    bool writeallrallypoints();
#endif

    /**
     * @brief Request geofence data to be written to log again
     * 
     * @return true if fence rewrite initiated, false if writer not in DONE state
     * 
     * @details Allows geofence data to be re-logged after changes without
     *          restarting entire logging system. Only works when writer
     *          is in DONE state.
     */
#if HAL_LOGGER_FENCE_ENABLED
    bool writeallfence();
#endif

private:

    /**
     * @brief Check if process time budget has been exceeded
     * 
     * @param[in] start_us Timestamp (microseconds) when processing started
     * @return true if time limit exceeded and processing should yield
     * 
     * @details Enforces time limits to prevent startup message writing
     *          from blocking logging thread excessively. Typically allows
     *          a few milliseconds per process() call.
     */
    static bool check_process_limit(uint32_t start_us);

    /**
     * @brief Stages for complete startup message sequence
     * 
     * Stage progression:
     * - FORMATS: Write all FMT (format definition) messages for every log
     *            message type. Critical first step as these define how to
     *            decode all subsequent messages. Typically 50-100 FMT messages.
     * 
     * - UNITS: Write all UNIT messages defining physical units (meters, degrees,
     *          m/s, etc.) used in log fields. Required for proper data scaling
     *          and display in analysis tools.
     * 
     * - MULTIPLIERS: Write all MULT messages defining fixed-point multipliers
     *                for compact data representation (e.g., 1e-7 for GPS coordinates).
     * 
     * - FORMAT_UNITS: Write FMTU messages linking format fields to their units
     *                 and multipliers. Enables automatic unit conversion in tools.
     * 
     * - PARMS: Write all vehicle parameters as PARM messages. Can be 200+
     *          parameters taking several hundred milliseconds. Each parameter
     *          written as name/value pair for configuration reconstruction.
     * 
     * - VEHICLE_MESSAGES: Write vehicle-specific startup messages (version info,
     *                     system configuration, RC setup) via _writesysinfo sub-writer.
     * 
     * - RUNNING_SUBWRITERS: Execute mission/rally/fence sub-writers. Must be last
     *                       as these can be re-triggered after DONE state to re-log
     *                       updated navigation data mid-flight.
     * 
     * - DONE: All startup messages written, writer idle until reset or sub-writer
     *         re-trigger requested.
     * 
     * @note Stage order is critical: FMT must be first, PARMS should be early,
     *       RUNNING_SUBWRITERS must be last to allow selective re-logging.
     * 
     * @warning Do not reorder stages without careful analysis of log decoder
     *          requirements and sub-writer re-triggering logic.
     */
    enum class Stage {
        FORMATS = 0,
        UNITS,
        MULTIPLIERS,
        FORMAT_UNITS,
        PARMS,
        VEHICLE_MESSAGES,
        RUNNING_SUBWRITERS, // must be last thing to run as we can redo bits of these
        DONE,
    };

    bool _fmt_done;      ///< True when format messages complete
    bool _params_done;   ///< True when parameter messages complete

    Stage stage;  ///< Current stage in startup sequence

    uint16_t next_format_to_send;  ///< Index of next FMT message to write

    uint8_t _next_unit_to_send;         ///< Index of next UNIT message to write
    uint8_t _next_format_unit_to_send;  ///< Index of next FMTU message to write
    uint8_t _next_multiplier_to_send;   ///< Index of next MULT message to write

    AP_Param::ParamToken token;  ///< Parameter iteration token for PARM stage
    AP_Param *ap;                ///< Current parameter being written
    float param_default;         ///< Default value of current parameter
    enum ap_var_type type;       ///< Type of current parameter

    /// Sub-writer for system information and version messages
    LoggerMessageWriter_WriteSysInfo _writesysinfo;

#if AP_MISSION_ENABLED
    /// Sub-writer for mission waypoint data
    LoggerMessageWriter_WriteEntireMission _writeentiremission;
#endif

#if HAL_RALLY_ENABLED
    /// Sub-writer for rally point locations
    LoggerMessageWriter_WriteAllRallyPoints _writeallrallypoints;
#endif

#if HAL_LOGGER_FENCE_ENABLED
    /// Sub-writer for geofence polygon data
    LoggerMessageWriter_Write_Polyfence _writeallpolyfence;
#endif
};
