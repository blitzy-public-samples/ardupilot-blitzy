# Building ArduPilot #

## Get the Source

Clone the project from GitHub:
```sh
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

You can also read more about the build system in the
[Waf Book](https://waf.io/book/).

waf should always be called from the locally cloned ardupilot root directory for the local branch you are trying to build from.

**Note**
Do not run `waf` with `sudo`!  This leads to permission and environment problems.

## Basic usage ##

There are several commands in the build system for advanced usage, but here we
list some basic and more used commands as example.

* **Build ArduCopter**

    Below shows how to build ArduCopter for the Pixhawk2/Cube. Many other boards are
    supported and the next section shows how to get a full list of them.

    ```sh
    ./waf configure --board CubeBlack
    ./waf copter
    ```

    The first command should be called only once or when you want to change a
    configuration option. One configuration often used is the `--board` option to
    switch from one board to another one. For example we could switch to
    SkyViper GPS drone and build again:

    ```sh
    ./waf configure --board skyviper-v2450
    ./waf copter
    ```

    If building for the bebop2 the binary must be built statically:

    ```sh
    ./waf configure --board bebop --static
    ./waf copter
    ```    

    The "arducopter" binary should appear in the `build/<board-name>/bin` directory.

* **List available boards**


    It's possible to get a list of supported boards on ArduPilot with the command
    below

    ```sh
    ./waf list_boards

    ```

    Here are some commands to configure waf for commonly used boards:

    ```sh
    ./waf configure --board bebop --static # Bebop or Bebop2
    ./waf configure --board edge           # emlid edge
    ./waf configure --board fmuv3          # 3DR Pixhawk 2 boards
    ./waf configure --board navio2         # emlid navio2
    ./waf configure --board Pixhawk1       # Pixhawk1
    ./waf configure --board CubeBlack      # Hex/ProfiCNC Cube Black (formerly known as Pixhawk 2.1)
    ./waf configure --board Pixracer       # Pixracer
    ./waf configure --board skyviper-v2450 # SkyRocket's SkyViper GPS drone using ChibiOS
    ./waf configure --board sitl           # software-in-the-loop simulator
    ./waf configure --board sitl --debug   # software-in-the-loop simulator with debug symbols

    ```

* **List of available vehicle types**

    Here is a list of the most common vehicle build targets:

    ```sh
    ./waf copter                            # All multirotor types
    ./waf heli                              # Helicopter types
    ./waf plane                             # Fixed wing airplanes including VTOL
    ./waf rover                             # Ground-based rovers and surface boats
    ./waf sub                               # ROV and other submarines
    ./waf antennatracker                    # Antenna trackers
    ./waf AP_Periph                         # AP Peripheral
    
    ```

* **Clean the build**

    Commands `clean` and `distclean` can be used to clean the objects produced by
    the build. The first keeps the `configure` information, cleaning only the
    objects for the current board. The second cleans everything for every board,
    including the saved `configure` information.

    Cleaning the build is very often not necessary and discouraged. We do
    incremental builds reducing the build time by orders of magnitude.

    If submodules are failing to be synchronized, `submodulesync` may be used
    to resync the submodules. This is usually necessary when shifting development
    between stable releases or a stable release and the master branch.

    In some some cases `submodule_force_clean` may be necessary. This removes all submodules and then performs a `submodulesync`. (Note whitelisted modules like esp_idf is not removed.)

* **Upload or install**

    Build commands have a `--upload` option in order to upload the binary built
    to a connected board. This option is supported by Pixhawk and Linux-based boards.
    The command below uses the `--targets` option that is explained in the next item.

    ```sh
    ./waf --targets bin/arducopter --upload
    ```

    For Linux boards you need first to configure the IP of the board you
    are going to upload to. This is done on configure phase with:

    ```sh
    ./waf configure --board <board> --rsync-dest <destination>
    ```

    The commands below give a concrete example (board and destination
    IP will change according to the board used):

    ```sh
    ./waf configure --board navio2 --rsync-dest root@192.168.1.2:/
    ./waf --target bin/arducopter --upload
    ```

    This allows to set a destination to which the `--upload` option will upload
    the binary.  Under the hood  it installs to a temporary location and calls
    `rsync <temp_install_location>/ <destination>`.

    On Linux boards there's also an install command, which will install to a certain
    directory, just like the temporary install above does. This can be
    used by distributors to create .deb, .rpm or other package types:

    ```sh
    ./waf copter
    DESTDIR=/my/temporary/location ./waf install
    ```

* **Use different targets**

    The build commands in the items above use `copter` as argument. This
    builds all binaries that fall under the "copter" group. See the
    section [Advanced usage](#advanced-usage) below for more details regarding
    groups.

    This shows a list of all possible targets:

    ```
    ./waf list
    ```

    For example, to build only a single binary:

    ```
    # Quad frame of ArduCopter
    ./waf --targets bin/arducopter

    # unit test of our math functions
    ./waf --targets tests/test_math
    ```

* **Use clang instead of gcc**

    Currently, gcc is the default on linux, and clang is used for MacOS.
    Building with clang on linux can be accomplished by setting the CXX
    environment variables during the configure step, e.g.:

    ```
    CXX=clang++ CC=clang ./waf configure --board=sitl
    ```

    Note: Your clang binary names may differ.

* **Other options**

    It's possible to see all available commands and options:

    ```
    ./waf -h
    ```

    Also, take a look on the [Advanced section](#advanced-usage) below.

### Using Docker ###

A docker environment is provided which may be helpful for building in a clean
environment and avoiding modification of the host environment.

To build the docker image (should only need to be done once), run:

```bash
docker build --rm -t ardupilot-dev .
```

To build inside the container, prefix your `waf` commands, e.g.:

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=sitl
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf copter
```

Alternatively, simply run `docker run --rm -it -v $PWD:/ardupilot ardupilot-dev` to
start a `bash` shell in which you can run other commands from this document.

## Advanced usage ##

This section contains some explanations on how the Waf build system works
and how you can use more advanced features.

Waf build system is composed of commands. For example, the command below
(`configure`) is for configuring the build with all the options used by this
particular build.

```bash
# Configure the Linux board
./waf configure --board=linux
```

Consequently, in order to build, a "build" command is issued, thus `waf build`.
That is the default command, so calling just `waf` is enough:

```bash
# Build programs from bin group
./waf

# Waf also accepts '-j' option to parallelize the build.
./waf -j8
```

By default waf tries to parallelize the build automatically to all processors
so the `-j` option is usually not needed, unless you are using icecc (thus
you want a bigger value) or you don't want to stress your machine with
the build.

### Program groups ###

Program groups are used to represent a class of programs. They can be used to
build all programs of a certain class without having to specify each program.
It's possible for two groups to overlap, except when both groups are main
groups. In other words, a program can belong to more than one group, but only
to one main group.

There's a special group, called "all", that comprises all programs.

#### Main groups ####

The main groups form a partition of all programs. Besides separating the
programs logically, they also define where they are built.

The main groups are:

 - bin: *the main binaries, that is, ardupilot's main products - the vehicles and
   Antenna Tracker*
 - tools
 - examples: *programs that show how certain libraries are used or to simply
   test their operation*
 - benchmarks: *requires `--enable-benchmarks` during configurarion*
 - tests: *basically unit tests to ensure changes don't break the system's
   logic*

All build files are placed under `build/<board>/`, where `<board>` represents
the board/platform you selected during configuration. Each main program group
has a folder with its name directly under `build/<board>/`. Thus, a program
will be stored in `build/<board>/<main_group>/`, where `<main_group>` is the
main group the program belongs to. For example, for a linux build, arduplane,
which belongs to the main group "bin", will be located at
`build/linux/bin/arduplane`.

#### Main product groups ####

Those are groups for ardupilot's main products. They contain programs for the
product they represent. Currently only the "copter" group has more than one
program - one for each frame type.

The main product groups are:

 - antennatracker
 - copter
 - plane
 - rover

#### Building a program group ####

Ardupilot adds to waf an option called `--program-group`, which receives as
argument the group you want it to build. For a build command, if you don't pass
any of `--targets` or `--program-group`, then the group "bin" is selected by
default. The option `--program-group` can be passed multiple times.

Examples:

```bash
# Group bin is the default one
./waf

# Build all vehicles and Antenna Tracker
./waf --program-group bin

# Build all benchmarks and tests
./waf --program-group benchmarks --program-group tests
```
#### Shortcut for program groups ####

For less typing, you can use the group name as the command to waf. Examples:

```bash
# Build all vehicles and Antenna Tracker
./waf bin

# Build all examples
./waf examples

# Build arducopter binaries
./waf copter
```

### Building a specific program ###

In order to build a specific program, you just need to pass its path relative
to `build/<board>/` to the option `--targets`. Example:

```bash
# Build arducopter for quad frame
./waf --targets bin/arducopter

# Build vectors unit test
./waf --targets tests/test_vectors
```

### Checking ###

The command `check` builds all programs and then executes the relevant tests.
In that context, a relevant test is a program from the group "tests" that makes
one of the following statements true:

 - it's the first time the test is built since the last cleanup or when the
   project was cloned.
 - the program had to be rebuilt (due to modifications in the code or
   dependencies, for example)
 - the test program failed in the previous check.

That is, the tests are run only if necessary. If you want waf to run all tests,
then you can use either option `--alltests` or the shortcut command
`check-all`.

Examples:

```bash
# Build everything and run relevant tests
./waf check

# Build everything and run all tests
./waf check --alltests

# Build everything and run all tests
./waf check-all
```

### Debugging ###

It's possible to pass the option `--debug` to the `configure` command. That
will set compiler flags to store debugging information in the binaries so that
you can use them with `gdb`, for example. That option might come handy when using SITL.

### Build-system wrappers ###

The `waf` binary on root tree is actually a wrapper to the real `waf` that's
maintained in its own submodule.  It's possible to call the latter directly via
`./modules/waf/waf-light` or to use an alias if you prefer typing `waf` over
`./waf`.

```sh
alias waf="<ardupilot-directory>/modules/waf/waf-light"

```

There's also a make wrapper called `Makefile.waf`. You can use
`make -f Makefile.waf help` for instructions on how to use it.

### Command line help ###

You can use `waf --help` to see information about commands and options built-in
to waf as well as some quick help on those added by ardupilot.

## Feature Flag System ##

ArduPilot's build system supports enabling and disabling features at compile time to optimize binary size and capabilities for different use cases. Features can be controlled using `--enable-<feature>` and `--disable-<feature>` flags during the configure phase.

### Enabling and Disabling Features ###

Feature flags are specified during the `./waf configure` command:

```sh
# Enable scripting support
./waf configure --board=CubeBlack --enable-scripting

# Disable FrSky telemetry to save flash space
./waf configure --board=CubeBlack --disable-frsky

# Multiple feature flags can be combined
./waf configure --board=CubeBlack --enable-scripting --disable-frsky --enable-dds
```

### Available Build Options ###

A comprehensive list of available build options is maintained in `Tools/scripts/build_options.py`. Common feature flags include:

**Communication Protocols:**
- `--enable-scripting` / `--disable-scripting` - Lua scripting support
- `--enable-dds` / `--disable-dds` - DDS/ROS2 integration via Micro-XRCE
- `--enable-frsky` / `--disable-frsky` - FrSky telemetry protocol
- `--enable-crsf` / `--disable-crsf` - CRSF (Crossfire) RC protocol
- `--enable-msp` / `--disable-msp` - MSP protocol for OSD

**Sensor Support:**
- `--enable-opticflow` / `--disable-opticflow` - Optical flow sensors
- `--enable-rangefinder` / `--disable-rangefinder` - Distance sensors
- `--enable-airspeed` / `--disable-airspeed` - Airspeed sensors (fixed-wing)

**Advanced Features:**
- `--enable-smart-battery` / `--disable-smart-battery` - Smart battery monitoring
- `--enable-beacon` / `--disable-beacon` - Beacon positioning
- `--enable-visualodom` / `--disable-visualodom` - Visual odometry

**Debugging and Development:**
- `--enable-debug` - Build with debug symbols for use with gdb
- `--enable-coverage` - Enable code coverage instrumentation
- `--enable-benchmarks` - Build benchmark programs

To see all available options, examine the build_options.py file or consult the board's hwdef file.

### Impact on Binary Size ###

Feature flags directly affect the compiled binary size. Disabling unused features can free up flash memory for boards with limited storage:

```sh
# Minimal build for small flash boards
./waf configure --board=MatekF405 --disable-scripting --disable-frsky \
    --disable-beacon --disable-visualodom

# Full-featured build for boards with ample flash
./waf configure --board=CubeOrange --enable-scripting --enable-dds \
    --enable-opticflow --enable-beacon
```

**Note:** Some features are board-specific and may be enabled or disabled by default in the board's hardware definition file (hwdef). Check `libraries/AP_HAL_ChibiOS/hwdef/<board>/hwdef.dat` for board defaults.

### Feature Dependencies ###

Some features have dependencies on other features or libraries. The build system will automatically enable required dependencies or report an error if incompatible options are selected.

## Cross-Compilation Setup ##

ArduPilot supports multiple target platforms requiring different toolchains for cross-compilation. This section covers toolchain setup for various target architectures.

### ARM Embedded Targets (STM32, etc.) ###

Most flight controller boards use ARM Cortex-M processors and require the ARM embedded toolchain.

**Installing gcc-arm-none-eabi on Linux:**

```sh
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi

# Verify installation
arm-none-eabi-gcc --version
```

**Installing on macOS:**

```sh
# Using Homebrew
brew install gcc-arm-embedded

# Or download from ARM's official site
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm
```

**Recommended Version:** ArduPilot is tested with gcc-arm-none-eabi version 10.2 or later. Older versions may produce incompatible binaries.

**Building for ARM targets:**

```sh
./waf configure --board=CubeBlack
./waf copter
```

The build system automatically detects and uses the ARM toolchain for embedded boards.

### ARM Linux Targets ###

Linux-based boards (Navio2, BeagleBone, etc.) require cross-compilation for ARM Linux.

**Installing ARM Linux cross-compiler:**

```sh
# For ARMv7 (32-bit)
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

# For ARMv8 (64-bit)
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

**Building for Linux boards:**

```sh
# Navio2 (ARMv7)
./waf configure --board=navio2
./waf copter

# Board selection automatically configures the appropriate toolchain
```

### ESP32 Targets ###

ESP32 boards require the ESP-IDF (Espressif IoT Development Framework) toolchain.

**Installing ESP-IDF:**

ArduPilot includes ESP-IDF as a submodule. Follow these steps:

```sh
# Update submodules to get ESP-IDF
git submodule update --init --recursive modules/esp_idf

# Install ESP-IDF prerequisites (Python packages)
cd modules/esp_idf
./install.sh esp32,esp32s3,esp32c3
cd ../..

# Source ESP-IDF environment (required for each build session)
source modules/esp_idf/export.sh
```

**Building for ESP32:**

```sh
./waf configure --board=esp32buzz
./waf copter
```

**Note:** ESP-IDF must be sourced in your shell environment before each build. Consider adding the export command to your shell profile for convenience.

### SITL (Software In The Loop) ###

SITL builds run natively on your development machine and don't require cross-compilation.

**Prerequisites:**

```sh
# Ubuntu/Debian
sudo apt-get install python3-dev python3-pip

# macOS
brew install python3

# Install MAVProxy and dependencies
pip3 install --user pymavlink MAVProxy
```

**Building SITL:**

```sh
./waf configure --board=sitl
./waf copter
```

SITL builds use your system's native compiler (gcc or clang) and produce binaries that run directly on your development machine.

### Toolchain Troubleshooting ###

**Problem:** `arm-none-eabi-gcc: command not found`
- **Solution:** ARM toolchain not installed or not in PATH. Install gcc-arm-none-eabi and verify with `arm-none-eabi-gcc --version`

**Problem:** `undefined reference` errors during linking
- **Solution:** Toolchain version mismatch. Ensure gcc-arm-none-eabi version 10.2 or later is installed

**Problem:** ESP-IDF errors about missing environment
- **Solution:** Source ESP-IDF environment: `source modules/esp_idf/export.sh`

## Board Selection Details ##

Understanding the board selection system helps you choose the right target and understand board capabilities.

### Board Definition System ###

Board configurations are defined in hardware definition (hwdef) files located in:

```
libraries/AP_HAL_ChibiOS/hwdef/<board-name>/
```

Each board directory contains:
- `hwdef.dat` - Main hardware definition file (pin assignments, peripherals, features)
- `hwdef-bl.dat` - Bootloader-specific configuration (if applicable)
- `README.md` - Board-specific documentation and capabilities

### Finding Board Capabilities ###

To understand what a board supports, examine its hwdef file:

```sh
# View board configuration
cat libraries/AP_HAL_ChibiOS/hwdef/CubeBlack/hwdef.dat

# Key information in hwdef files:
# - MCU type and speed
# - Flash and RAM size
# - Available serial ports (UART/USART)
# - SPI and I2C buses
# - PWM output channels
# - Default enabled/disabled features
```

### Board Families ###

ArduPilot supports several board families with common characteristics:

**Pixhawk Family:**
- `Pixhawk1` - Original Pixhawk (STM32F427)
- `fmuv3` - Pixhawk 2 (STM32F427)
- `Pixhawk4` - Pixhawk 4 (STM32F765)
- `CubeBlack` - Hex Cube Black / Pixhawk 2.1 (STM32F427)
- `CubeOrange` - Hex Cube Orange (STM32F777)
- `CubeOrange-ODID` - Cube Orange with OpenDroneID (STM32F777)
- `CubeOrangePlus` - Cube Orange+ with higher clock and more RAM (STM32H743)
- `CubeYellow` - Hex Cube Yellow (STM32F777)
- `CubePurple` - Hex Cube Purple (STM32H743)

**Holybro Boards:**
- `Pixhawk4` - Pixhawk 4 (STM32F765)
- `Pixhawk6C` - Pixhawk 6C (STM32H743)
- `Pixhawk6X` - Pixhawk 6X (STM32H743)

**mRo Boards:**
- `mRoPixracerPro` - mRo Pixracer Pro (STM32F427)
- `mRoControlZeroF7` - mRo Control Zero F7 (STM32F777)
- `mRoX21-777` - mRo X2.1-777 (STM32F777)

**Matek Boards:**
- `MatekF405` - Matek F405 series
- `MatekF405-Wing` - Optimized for fixed-wing
- `MatekH743` - Matek H743 series

**Linux Boards:**
- `navio2` - Emlid Navio2 (Raspberry Pi HAT)
- `edge` - Emlid Edge
- `bebop` - Parrot Bebop/Bebop2
- `linux` - Generic Linux target

**Simulation:**
- `sitl` - Software in the loop simulator
- `sitl-periph` - SITL for AP_Periph simulation

### Choosing the Right Board ###

Consider these factors when selecting a board:

1. **Flash Memory:** Boards with more flash (2MB+) support all features. Smaller flash boards (1MB) require disabling features.

2. **Processor:** Newer STM32H7 boards offer more performance than older STM32F4 boards.

3. **Peripheral Support:** Check hwdef for available UARTs, SPI buses, and I2C buses for your sensors.

4. **PWM Outputs:** Ensure the board has enough PWM channels for your vehicle configuration.

5. **Voltage:** Some boards operate at 3.3V, others at 5V. Verify compatibility with your peripherals.

### Example Board Selection ###

```sh
# Small quadcopter with basic features - use F405
./waf configure --board=MatekF405

# Advanced multicopter with LUA scripting and DDS - use H743
./waf configure --board=CubeOrangePlus

# Fixed-wing with long missions - use board optimized for Plane
./waf configure --board=MatekF405-Wing

# Development and testing - use SITL
./waf configure --board=sitl
```

## Troubleshooting Common Build Issues ##

This section covers common build problems and their solutions.

### Submodule Sync Failures ###

**Symptoms:**
- `error: No such file or directory` for files in `modules/`
- Missing ChibiOS, MAVLink, or other submodule files
- Build failures after switching branches

**Solutions:**

```sh
# Standard submodule sync (recommended first attempt)
./waf submodulesync

# If standard sync fails, use force clean
./waf submodule_force_clean

# Manual submodule update (if waf commands fail)
git submodule update --init --recursive

# For specific problematic submodule
git submodule update --init --recursive modules/ChibiOS
```

**When to use each:**
- `submodulesync` - First attempt, quick resync of submodules
- `submodule_force_clean` - When submodules are corrupted or mismatched
- Manual `git submodule` - When waf submodule commands fail entirely

**Note:** Switching between stable releases and master branch often requires `submodulesync` due to submodule version changes.

### Compiler Version Compatibility ###

**Symptoms:**
- `internal compiler error`
- Linker errors about incompatible object files
- Warnings about deprecated compiler flags

**Solutions:**

```sh
# Check ARM toolchain version
arm-none-eabi-gcc --version

# Required: gcc-arm-none-eabi 10.2 or later
# If version is too old, update toolchain:

# Ubuntu/Debian
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi

# Or download from ARM:
# https://developer.arm.com/downloads/-/gnu-rm
```

**Known Issues:**
- gcc-arm-none-eabi 6.x and older: Unsupported, produces incompatible code
- gcc-arm-none-eabi 8.x: May work but not recommended
- gcc-arm-none-eabi 10.2+: Recommended and tested

### Missing Dependencies ###

**Symptoms:**
- `ImportError: No module named pymavlink`
- `fatal error: Python.h: No such file or directory`
- `command not found` errors for build tools

**Solutions:**

```sh
# Install Python development headers
sudo apt-get install python3-dev python3-pip

# Install Python dependencies
pip3 install --user pymavlink future lxml

# Install build essentials
sudo apt-get install build-essential

# For SITL, install additional dependencies
pip3 install --user MAVProxy

# Verify installations
python3 -c "import pymavlink; print('OK')"
```

**Environment Setup Script:**

ArduPilot provides environment setup scripts for common platforms:

```sh
# Run environment setup for your platform
./Tools/environment_install/install-prereqs-ubuntu.sh

# For other platforms, see:
# ./Tools/environment_install/install-prereqs-*.sh
```

### Build Cache Corruption ###

**Symptoms:**
- Strange build errors that don't match the code
- Errors that persist after fixing obvious issues
- Inconsistent build failures

**When to use `clean` vs `distclean`:**

```sh
# clean - Removes build objects for current board, keeps configuration
# Use when: Code changes aren't being picked up
./waf clean
./waf copter

# distclean - Removes ALL build artifacts for ALL boards
# Use when: Build cache is corrupted, switching projects, or configuration issues
./waf distclean
./waf configure --board=CubeBlack
./waf copter
```

**Recommendation:** Start with `./waf clean`. Only use `./waf distclean` if problems persist, as it requires reconfiguration.

### Permission Errors ###

**Symptoms:**
- `Permission denied` when writing build files
- `cannot create directory` errors
- Build files owned by root

**Solutions:**

```sh
# NEVER run waf with sudo
# If you accidentally ran with sudo, fix permissions:
sudo chown -R $USER:$USER build/

# Remove any root-owned build artifacts
sudo rm -rf build/
./waf configure --board=<your-board>
```

**Important:** Running `./waf` with `sudo` causes permission problems and environment issues. Always run waf as a regular user.

### Out of Flash Space ###

**Symptoms:**
- `region 'isr_vector' overflowed`
- `region 'flash' overflowed`
- `will not fit in region`

**Solutions:**

```sh
# Disable unnecessary features to reduce binary size
./waf configure --board=MatekF405 --disable-scripting --disable-frsky \
    --disable-beacon --disable-visualodom

# Check which features are enabled in board's hwdef
cat libraries/AP_HAL_ChibiOS/hwdef/<board>/hwdef.dat | grep "define HAL"

# Consider using a board with more flash memory
# F405: 1MB flash
# F765: 2MB flash
# H743: 2MB flash
```

### ESP-IDF Environment Issues ###

**Symptoms:**
- `idf.py: command not found`
- ESP32 build fails with missing toolchain
- Version mismatch errors

**Solutions:**

```sh
# Ensure ESP-IDF submodule is initialized
git submodule update --init --recursive modules/esp_idf

# Install ESP-IDF toolchain
cd modules/esp_idf
./install.sh esp32,esp32s3,esp32c3
cd ../..

# Source environment (required each session)
source modules/esp_idf/export.sh

# Add to your ~/.bashrc for persistence:
# alias esp-idf-export='source ~/ardupilot/modules/esp_idf/export.sh'
```

### Python Version Issues ###

**Symptoms:**
- `SyntaxError` in build scripts
- `invalid syntax` in Python code
- Feature detection failures

**Solutions:**

```sh
# ArduPilot requires Python 3.6 or later
python3 --version

# If Python version is too old, update:
# Ubuntu 18.04+
sudo apt-get install python3.8

# Ensure pip is for Python 3
pip3 --version

# Set Python 3 as default (if needed)
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
```

## Documentation Generation ##

ArduPilot includes comprehensive API documentation generated from source code using Doxygen.

### Building API Documentation ###

Generate complete documentation for all components:

```sh
# Build all documentation (libraries + all vehicles)
./Tools/scripts/build_docs.sh
```

The documentation will be generated in `$HOME/build/ArduPilot-docs/` by default (or the directory specified by `$DOCS_OUTPUT_BASE` environment variable).

### Component-Specific Documentation ###

Generate documentation for specific components:

```sh
# Library documentation only
./docs/build-libs.sh

# ArduCopter documentation
./docs/build-arducopter.sh

# ArduPlane documentation
./docs/build-arduplane.sh

# Rover documentation
./docs/build-apmrover2.sh

# ArduSub documentation
./docs/build-ardusub.sh
```

### Viewing Generated Documentation ###

After building, open the HTML documentation in your browser:

```sh
# Library documentation
xdg-open $HOME/build/ArduPilot-docs/libraries/html/index.html

# Vehicle-specific documentation
xdg-open $HOME/build/ArduPilot-docs/arducopter/html/index.html
```

On macOS, use `open` instead of `xdg-open`.

### Documentation Prerequisites ###

Ensure documentation tools are installed:

```sh
# Ubuntu/Debian
sudo apt-get install doxygen graphviz

# macOS
brew install doxygen graphviz

# Verify installation
doxygen --version  # Should be 1.9.8 or later
```

### Detailed Documentation Instructions ###

For comprehensive documentation generation and contribution guidelines, see:

```
docs/README.md
```

This includes:
- Documentation structure and organization
- Doxygen comment style guide
- README file templates
- Diagram creation with Mermaid
- Documentation quality standards

## Build Output Locations ##

Understanding where the build system places output files helps with deployment and debugging.

### Binary Output ###

Compiled binaries are placed in the `bin/` subdirectory of each board's build directory:

```
build/<board-name>/bin/
```

**Examples:**

```sh
# ArduCopter for CubeBlack
build/CubeBlack/bin/arducopter.apj       # Firmware for uploading via GCS
build/CubeBlack/bin/arducopter           # ELF binary with debug symbols
build/CubeBlack/bin/arducopter.hex       # Intel HEX format

# ArduPlane for SITL
build/sitl/bin/arduplane                 # Native executable for simulation

# Multiple frame types for copter
build/CubeBlack/bin/arducopter          # Default (quad)
build/CubeBlack/bin/arducopter-hexa     # Hexacopter
build/CubeBlack/bin/arducopter-octa     # Octacopter
```

**File Extensions:**
- `.apj` - ArduPilot JSON firmware format (used by ground control stations for upload)
- `.hex` - Intel HEX format (used by some upload tools)
- `.bin` - Raw binary format
- `.elf` - ELF format with debug symbols (for use with gdb)
- No extension - ELF binary (Linux/SITL) or same as `.elf` (embedded)

### ROMFS Data ###

Read-only filesystem data (scripts, configuration files) is placed in:

```
build/<board-name>/romfs/
```

This includes:
- Lua scripts (if scripting is enabled)
- Default parameter files
- Embedded configuration data

### Intermediate Build Objects ###

Object files, dependency files, and other intermediate build artifacts:

```
build/<board-name>/<library-or-program>/
```

**Examples:**

```sh
# Library object files
build/CubeBlack/libraries/AP_HAL_ChibiOS/

# Program-specific objects
build/CubeBlack/arducopter/

# Test program objects
build/sitl/tests/test_vectors/
```

**Note:** These intermediate files are managed by the build system. You typically don't need to interact with them directly.

### Build Configuration ###

Build configuration and cache files:

```
build/<board-name>/
  c4che/              # Configuration cache (waf internal)
  compile_commands.json  # Compilation database for IDEs
```

The `compile_commands.json` file is useful for IDE integration (VS Code, CLion, etc.) and provides IntelliSense with accurate include paths and compiler flags.

### Cleaning Build Outputs ###

```sh
# Remove build objects for current board (keeps configuration)
./waf clean

# Remove ALL build artifacts for ALL boards
./waf distclean

# Remove build directory manually (if needed)
rm -rf build/
```

### Upload Locations ###

When using `--upload`, binaries are copied to the target:

**For flight controllers:**
- Upload via USB bootloader
- Binary is written directly to flash memory
- No filesystem location (embedded system)

**For Linux boards:**
```
# Default installation prefix
/usr/bin/ardupilot/

# Custom destination (if configured with --rsync-dest)
<destination-path>/usr/bin/ardupilot/
```

### Accessing Build Artifacts ###

**Get path to built binary:**

```sh
# Find the binary for your board
ls build/CubeBlack/bin/

# Run SITL binary directly
./build/sitl/bin/arducopter --help
```

**Upload manually:**

```sh
# Upload using your configured method
./waf copter --upload

# Or use GCS (Mission Planner, QGroundControl) to upload the .apj file
# File: build/<board>/bin/arducopter.apj
```
