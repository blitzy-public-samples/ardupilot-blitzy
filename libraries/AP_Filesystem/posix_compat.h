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
 * @file posix_compat.h
 * @brief POSIX-compatible FILE* API for platforms without standard C library
 * 
 * @details This header provides a POSIX-compatible file I/O interface on platforms
 *          that lack a standard C library, particularly ChibiOS-based boards.
 *          
 *          **Primary Use Case**: Lua scripting integration requiring stdio-like interface
 *          
 *          **Architecture**: Unbuffered wrapper around AP::FS() singleton, providing
 *          consistent behavior across all ArduPilot platforms (ChibiOS, Linux, ESP32, SITL).
 *          
 *          **Design Decision**: Unbuffered I/O implementation (no internal buffering like
 *          standard FILE*) for simplicity and predictable behavior on embedded systems.
 *          
 *          **Cross-Platform Consistency**: Same API behavior on all supported platforms,
 *          enabling portable C code (particularly Lua scripting engine).
 * 
 * @note This layer is unbuffered - no internal buffering like standard FILE*
 * @note Designed for Lua scripting compatibility on ChibiOS platforms
 * @note C++ code should prefer AP::FS() native API for better performance
 * @note Thread safety provided by underlying AP::FS() backend semaphores
 * @note Standard streams (stdin/stdout/stderr) are placeholders, not functional
 * 
 * @warning Only one ungetc() pushback character supported (standard allows unlimited)
 * @warning No internal buffering (performance implications for small reads/writes)
 * @warning Standard streams are non-functional placeholders
 * @warning Temporary files not truly anonymous (visible in filesystem)
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <sys/types.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief File handle structure for POSIX compatibility layer
 * 
 * @details This structure wraps an AP::FS() file descriptor with stdio-like
 *          state tracking to provide POSIX FILE* semantics. Primarily used
 *          to enable Lua scripting on ChibiOS platforms that lack a standard
 *          C library.
 *          
 *          Each APFS_FILE represents an open file and maintains error/EOF state
 *          independently of the underlying file descriptor.
 * 
 * @note Thread safety: File handles should not be shared between threads
 * @note Lifecycle: Created by apfs_fopen()/apfs_tmpfile(), destroyed by apfs_fclose()
 */
typedef struct apfs_file {
    /**
     * @brief Underlying AP::FS() file descriptor
     * @details Set to -1 when the file is closed. All I/O operations are
     *          performed through this descriptor using AP::FS() singleton.
     */
    int fd;
    
    /**
     * @brief Error flag for ferror() implementation
     * @details Set to true when an I/O error occurs. Cleared by clearerr().
     */
    bool error;
    
    /**
     * @brief End-of-file flag for feof() detection
     * @details Set to true when a read operation encounters EOF.
     *          Cleared by clearerr() or successful seek operations.
     */
    bool eof;
    
    /**
     * @brief Pushback character for ungetc() implementation
     * @details Stores a single pushed-back character. Set to -1 when no
     *          character is pushed back. Only one character pushback supported.
     * @warning Limited to one character (standard C allows unlimited pushback)
     */
    int16_t unget;
    
    /**
     * @brief Dynamically allocated temporary file path
     * @details Non-NULL only for files created with apfs_tmpfile().
     *          The file at this path is deleted when apfs_fclose() is called.
     *          NULL for regular files opened with apfs_fopen().
     */
    char *tmpfile_name;
} APFS_FILE;

/*
 * File Operations
 */

/**
 * @brief Open file with stdio-like mode string
 * 
 * @details Translates POSIX mode strings to AP::FS() flags and opens the file.
 *          Allocates an APFS_FILE structure to track file state.
 *          
 *          Supported modes:
 *          - "r": Read only, file must exist
 *          - "w": Write only, truncate existing or create new
 *          - "a": Append, create if doesn't exist
 *          - "r+": Read/write, file must exist
 *          - "w+": Read/write, truncate existing or create new
 *          - "a+": Read/append, create if doesn't exist
 * 
 * @param[in] pathname File path (absolute or relative to filesystem root)
 * @param[in] mode POSIX open mode string (e.g., "r", "w", "a", "r+", "w+", "a+")
 * 
 * @return File handle on success, NULL on error (file not found, permission denied, etc.)
 * 
 * @note Unbuffered I/O - no internal buffering performed
 * @note Thread-safe: Multiple threads can open different files concurrently
 * 
 * @see apfs_fclose()
 */
APFS_FILE *apfs_fopen(const char *pathname, const char *mode);

/**
 * @brief Close file and free handle
 * 
 * @details Closes the underlying AP::FS() file descriptor and deallocates
 *          the APFS_FILE structure. For temporary files created with
 *          apfs_tmpfile(), also deletes the file from the filesystem.
 * 
 * @param[in] stream File handle to close
 * 
 * @return 0 on success, EOF (-1) on error
 * 
 * @note After closing, the stream pointer is invalid and must not be used
 * @note Temporary files are automatically deleted from filesystem
 * 
 * @see apfs_fopen(), apfs_tmpfile()
 */
int apfs_fclose(APFS_FILE *stream);

/**
 * @brief Close and reopen file with new path/mode
 * 
 * @details Closes the file associated with stream and reopens it with
 *          a new pathname and mode, reusing the same APFS_FILE structure.
 *          If the reopen fails, the stream is still closed.
 * 
 * @param[in] pathname New file path to open
 * @param[in] mode New open mode string
 * @param[in,out] stream Existing file handle to reuse
 * 
 * @return Reopened file handle (same as stream parameter) on success, NULL on error
 * 
 * @note On failure, the original file is still closed
 * 
 * @see apfs_fopen(), apfs_fclose()
 */
APFS_FILE *apfs_freopen(const char *pathname, const char *mode, APFS_FILE *stream);

/**
 * @brief Create temporary file
 * 
 * @details Creates a temporary file with a unique generated name in the
 *          filesystem. The file is opened in "w+" mode (read/write, truncate).
 *          The file is automatically deleted when apfs_fclose() is called.
 * 
 * @return File handle to temporary file on success, NULL on error
 * 
 * @warning Temporary files are NOT truly anonymous - they are visible in
 *          the filesystem until closed
 * @note Temporary file paths follow pattern: /tmp/apfs_tmpXXXXXX
 * @note File is automatically deleted on close
 * 
 * @see apfs_fclose()
 */
APFS_FILE *apfs_tmpfile(void);

/*
 * Read Operations
 */

/**
 * @brief Read binary data from file
 * 
 * @details Reads up to nmemb elements of size bytes each from the file.
 *          Returns the number of complete elements successfully read.
 *          May return fewer elements than requested on EOF or error.
 * 
 * @param[out] ptr Destination buffer to receive data
 * @param[in] size Size of each element in bytes
 * @param[in] nmemb Number of elements to read
 * @param[in,out] stream File handle to read from
 * 
 * @return Number of elements successfully read (0 to nmemb)
 *         May be less than nmemb on EOF or error. Check feof()/ferror() to distinguish.
 * 
 * @note Unbuffered - directly reads from underlying file descriptor
 * @note Sets EOF flag if end of file reached
 * @note Sets error flag on I/O error
 * 
 * @see apfs_fwrite(), apfs_feof(), apfs_ferror()
 */
size_t apfs_fread(void *ptr, size_t size, size_t nmemb, APFS_FILE *stream);

/**
 * @brief Read line into buffer
 * 
 * @details Reads characters from the file into buffer until:
 *          - A newline character is encountered (newline is included in buffer)
 *          - EOF is reached
 *          - size-1 characters have been read
 *          
 *          The string is always null-terminated.
 * 
 * @param[out] s Destination buffer to receive line
 * @param[in] size Maximum buffer size (including null terminator)
 * @param[in,out] stream File handle to read from
 * 
 * @return Pointer to buffer (s parameter) on success, NULL on EOF or error
 * 
 * @note Returns NULL on EOF even if some characters were read
 * @note Newline character is included in the buffer if encountered
 * @note Buffer is always null-terminated
 * 
 * @see apfs_fputs(), apfs_getc()
 */
char *apfs_fgets(char *s, int size, APFS_FILE *stream);

/**
 * @brief Read single character from file
 * 
 * @details Reads one character from the file. If a character was pushed
 *          back with ungetc(), returns that character instead.
 * 
 * @param[in,out] stream File handle to read from
 * 
 * @return Character read (0-255) on success, EOF (-1) on end-of-file or error
 * 
 * @note Returns pushed-back character if ungetc() was called
 * @note Sets EOF flag when end of file reached
 * 
 * @see apfs_ungetc(), apfs_feof()
 */
int apfs_getc(APFS_FILE *stream);

/**
 * @brief Push character back to stream
 * 
 * @details Pushes a character back onto the stream. The next read operation
 *          will return this character. Only one character can be pushed back.
 * 
 * @param[in] c Character to push back (0-255)
 * @param[in,out] stream File handle to push character to
 * 
 * @return Pushed character on success, EOF (-1) on error
 * 
 * @warning Only ONE character pushback supported (standard C allows unlimited)
 * @warning Calling ungetc() twice without an intervening read will fail
 * @note Clears EOF flag when character is successfully pushed back
 * 
 * @see apfs_getc()
 */
int apfs_ungetc(int c, APFS_FILE *stream);

/*
 * Write Operations
 */

/**
 * @brief Write binary data to file
 * 
 * @details Writes nmemb elements of size bytes each to the file.
 *          Returns the number of complete elements successfully written.
 * 
 * @param[in] ptr Source buffer containing data to write
 * @param[in] size Size of each element in bytes
 * @param[in] nmemb Number of elements to write
 * @param[in,out] stream File handle to write to
 * 
 * @return Number of elements successfully written (0 to nmemb)
 *         Less than nmemb indicates an error occurred.
 * 
 * @note Unbuffered - directly writes to underlying file descriptor
 * @note Sets error flag on I/O error
 * 
 * @see apfs_fread(), apfs_fflush()
 */
size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *stream);

/**
 * @brief Formatted output to file
 * 
 * @details Writes formatted output to the file using printf-style format string.
 *          Supports standard printf format specifiers.
 * 
 * @param[in,out] stream File handle to write to
 * @param[in] format Printf-style format string
 * @param[in] ... Variable arguments matching format specifiers
 * 
 * @return Number of characters written on success, negative value on error
 * 
 * @note Unbuffered - output is written immediately
 * @note Sets error flag on write failure
 * 
 * @see apfs_fputs()
 */
int apfs_fprintf(APFS_FILE *stream, const char *format, ...);

/**
 * @brief Write string to file
 * 
 * @details Writes a null-terminated string to the file.
 *          Does not append a newline character.
 * 
 * @param[in] s Null-terminated string to write
 * @param[in,out] stream File handle to write to
 * 
 * @return Non-negative value on success, EOF (-1) on error
 * 
 * @note Does not automatically append newline (unlike puts())
 * @note Sets error flag on write failure
 * 
 * @see apfs_fgets(), apfs_fprintf()
 */
int apfs_fputs(const char *s, APFS_FILE *stream);

/**
 * @brief Flush write buffers to disk
 * 
 * @details Forces any buffered data to be written to the underlying storage.
 *          In this unbuffered implementation, calls fsync() on the file descriptor.
 * 
 * @param[in,out] stream File handle to flush
 * 
 * @return 0 on success, EOF (-1) on error
 * 
 * @note Unbuffered implementation - calls fsync() to sync to physical storage
 * @note May block until physical write completes
 * 
 * @see apfs_fwrite()
 */
int apfs_fflush(APFS_FILE *stream);

/*
 * Positioning Operations
 */

/**
 * @brief Seek to position in file
 * 
 * @details Sets the file position indicator to the specified offset
 *          relative to the origin specified by whence.
 * 
 * @param[in,out] stream File handle to seek
 * @param[in] offset Byte offset from origin
 * @param[in] whence Origin for offset:
 *                   - SEEK_SET (0): Beginning of file
 *                   - SEEK_CUR (1): Current position
 *                   - SEEK_END (2): End of file
 * 
 * @return 0 on success, -1 on error
 * 
 * @note Clears EOF flag on successful seek
 * @note Discards any pushed-back character from ungetc()
 * 
 * @see apfs_ftell()
 */
int apfs_fseek(APFS_FILE *stream, long offset, int whence);

/**
 * @brief Get current file position
 * 
 * @details Returns the current byte offset from the beginning of the file.
 * 
 * @param[in] stream File handle to query
 * 
 * @return Current byte offset (0-based) on success, -1 on error
 * 
 * @note Does not account for pushed-back characters from ungetc()
 * 
 * @see apfs_fseek()
 */
long apfs_ftell(APFS_FILE *stream);

/*
 * Status and Error Handling Operations
 */

/**
 * @brief Check error flag
 * 
 * @details Tests whether the error indicator is set for the stream.
 *          The error indicator is set by I/O operations that fail.
 * 
 * @param[in] stream File handle to check
 * 
 * @return Non-zero if error indicator is set, zero otherwise
 * 
 * @note Error flag persists until cleared with clearerr()
 * 
 * @see apfs_clearerr(), apfs_feof()
 */
int apfs_ferror(APFS_FILE *stream);

/**
 * @brief Check end-of-file flag
 * 
 * @details Tests whether the end-of-file indicator is set for the stream.
 *          The EOF indicator is set when a read operation reaches end of file.
 * 
 * @param[in] stream File handle to check
 * 
 * @return Non-zero if EOF indicator is set, zero otherwise
 * 
 * @note EOF flag persists until cleared with clearerr() or a successful seek
 * 
 * @see apfs_ferror(), apfs_clearerr()
 */
int apfs_feof(APFS_FILE *stream);

/**
 * @brief Clear error and EOF flags
 * 
 * @details Clears both the error indicator and end-of-file indicator
 *          for the stream, allowing continued I/O operations.
 * 
 * @param[in,out] stream File handle to clear flags for
 * 
 * @note Does not close or reopen the file
 * @note Does not clear any system-level errors
 * 
 * @see apfs_ferror(), apfs_feof()
 */
void apfs_clearerr(APFS_FILE *stream);

/*
 * File System Operations
 */

/**
 * @brief Delete file from filesystem
 * 
 * @details Removes the specified file from the filesystem.
 *          Fails if file is currently open or doesn't exist.
 * 
 * @param[in] pathname Path to file to delete
 * 
 * @return 0 on success, -1 on error
 * 
 * @warning Fails if file is currently open
 * @note Implemented via AP::FS()->unlink()
 * 
 * @see apfs_rename()
 */
int apfs_remove(const char *pathname);

/**
 * @brief Rename or move file
 * 
 * @details Renames a file from oldpath to newpath. Can be used to move
 *          files between directories if supported by the filesystem.
 * 
 * @param[in] oldpath Current file path
 * @param[in] newpath New file path
 * 
 * @return 0 on success, -1 on error
 * 
 * @note Behavior is filesystem-dependent for cross-directory moves
 * @note Fails if source doesn't exist or destination already exists
 * 
 * @see apfs_remove()
 */
int apfs_rename(const char *oldpath, const char *newpath);

/*
 * Standard Stream Handles and Constants
 */

/**
 * @brief Standard input stream placeholder
 * @warning Non-functional placeholder - stdin is not supported in this implementation
 * @note Defined for API compatibility with standard C, but operations will fail
 */
#undef stdin
#define stdin ((APFS_FILE*)1)

/**
 * @brief Standard output stream placeholder
 * @warning Non-functional placeholder - stdout is not supported in this implementation
 * @note Defined for API compatibility with standard C, but operations will fail
 */
#undef stdout
#define stdout ((APFS_FILE*)2)

/**
 * @brief Standard error stream placeholder
 * @warning Non-functional placeholder - stderr is not supported in this implementation
 * @note Defined for API compatibility with standard C, but operations will fail
 */
#undef stderr
#define stderr ((APFS_FILE*)3)

/**
 * @brief Buffer size constant
 * @details Default buffer size used by some stdio operations.
 *          Set to 256 bytes for embedded systems.
 */
#undef BUFSIZ
#define BUFSIZ 256

/**
 * @brief End-of-file return value
 * @details Returned by read functions to indicate end-of-file or error.
 */
#define EOF (-1)

/**
 * @brief File positioning constant - beginning of file
 * @details Used with fseek() to position relative to start of file.
 */
#ifndef SEEK_SET
#define SEEK_SET 0

/**
 * @brief File positioning constant - current position
 * @details Used with fseek() to position relative to current offset.
 */
#define SEEK_CUR 1

/**
 * @brief File positioning constant - end of file
 * @details Used with fseek() to position relative to end of file.
 */
#define SEEK_END 2
#endif

/**
 * @brief Map standard FILE type to APFS_FILE
 * @details Transparently replaces standard FILE type with APFS_FILE
 *          for C code compatibility.
 * @note Enables Lua and other C code to use familiar FILE* API
 */
#define FILE APFS_FILE

#ifndef __cplusplus
/**
 * @brief Transparent replacement of standard C file functions for Lua compatibility
 * 
 * @details These macro definitions redirect standard POSIX file I/O functions
 *          to their apfs_* equivalents, enabling unmodified C code (particularly
 *          the Lua scripting engine) to compile and run on platforms without
 *          a standard C library (ChibiOS).
 *          
 *          **Design Decision**: Only active for C code, not C++. C++ code should
 *          use the AP::FS() native API directly for better performance and type safety.
 *          
 *          **Thread Safety**: Inherited from underlying AP::FS() backend implementations.
 *          File handles should not be shared between threads.
 * 
 * @note Only active when compiling as C code (not C++)
 * @note C++ code should use AP_Filesystem APIs directly
 * @note Enables Lua scripting engine to use familiar stdio API
 */

/**
 * @brief Redirect fopen() to apfs_fopen()
 * @details Transparent replacement of standard fopen() for C code
 */
#define fopen(p,m) apfs_fopen(p,m)

/**
 * @brief Redirect fprintf() to apfs_fprintf()
 * @details Transparent replacement of standard fprintf() for C code
 */
#define fprintf(stream, format, args...) apfs_fprintf(stream, format, ##args)

/**
 * @brief Redirect fflush() to apfs_fflush()
 * @details Transparent replacement of standard fflush() for C code
 */
#define fflush(s) apfs_fflush(s)

/**
 * @brief Redirect fread() to apfs_fread()
 * @details Transparent replacement of standard fread() for C code
 */
#define fread(ptr,size,nmemb, stream) apfs_fread(ptr, size, nmemb, stream)

/**
 * @brief Redirect fwrite() to apfs_fwrite()
 * @details Transparent replacement of standard fwrite() for C code
 */
#define fwrite(ptr, size, nmemb, stream) apfs_fwrite(ptr, size, nmemb, stream)

/**
 * @brief Redirect fputs() to apfs_fputs()
 * @details Transparent replacement of standard fputs() for C code
 */
#define fputs(s, stream) apfs_fputs(s, stream)

/**
 * @brief Redirect fgets() to apfs_fgets()
 * @details Transparent replacement of standard fgets() for C code
 */
#define fgets(s, size, stream) apfs_fgets(s, size, stream)

/**
 * @brief Redirect clearerr() to apfs_clearerr()
 * @details Transparent replacement of standard clearerr() for C code
 */
#define clearerr(stream) apfs_clearerr(stream)

/**
 * @brief Redirect fseek() to apfs_fseek()
 * @details Transparent replacement of standard fseek() for C code
 */
#define fseek(stream, offset, whence) apfs_fseek(stream, offset, whence)

/**
 * @brief Redirect ferror() to apfs_ferror()
 * @details Transparent replacement of standard ferror() for C code
 */
#define ferror(stream) apfs_ferror(stream)

/**
 * @brief Redirect fclose() to apfs_fclose()
 * @details Transparent replacement of standard fclose() for C code
 */
#define fclose(stream) apfs_fclose(stream)

/**
 * @brief Redirect tmpfile() to apfs_tmpfile()
 * @details Transparent replacement of standard tmpfile() for C code
 */
#define tmpfile() apfs_tmpfile()

/**
 * @brief Redirect getc() to apfs_getc()
 * @details Transparent replacement of standard getc() for C code
 */
#undef getc
#define getc(stream) apfs_getc(stream)

/**
 * @brief Redirect ungetc() to apfs_ungetc()
 * @details Transparent replacement of standard ungetc() for C code
 */
#define ungetc(c, stream) apfs_ungetc(c, stream)

/**
 * @brief Redirect feof() to apfs_ferror()
 * @details Transparent replacement of standard feof() for C code
 * @note Implementation currently uses ferror() - may need verification
 */
#define feof(stream) apfs_ferror(stream)

/**
 * @brief Redirect ftell() to apfs_ftell()
 * @details Transparent replacement of standard ftell() for C code
 */
#define ftell(stream) apfs_ftell(stream)

/**
 * @brief Redirect freopen() to apfs_freopen()
 * @details Transparent replacement of standard freopen() for C code
 */
#define freopen(pathname, mode, stream) apfs_freopen(pathname, mode, stream)

/**
 * @brief Redirect rename() to apfs_rename()
 * @details Transparent replacement of standard rename() for C code
 */
#define rename(oldpath, newpath) apfs_rename(oldpath, newpath)

#if !defined(__APPLE__)
/**
 * @brief Redirect remove() to apfs_remove()
 * @details Transparent replacement of standard remove() for C code
 * @note Not redefined on Apple platforms to avoid conflicts
 */
#define remove(pathname) apfs_remove(pathname)

/**
 * @brief Formatted string output declaration
 * @details Provided for C code compatibility on non-Apple platforms
 */
int sprintf(char *str, const char *format, ...);
#endif
#endif // __cplusplus

#ifdef __cplusplus
}
#endif
