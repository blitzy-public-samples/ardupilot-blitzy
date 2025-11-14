#!/usr/bin/env python3
# encoding: utf-8
# flake8: noqa

"""
ArduPilot Build System Configuration (waf)

This file is the main build configuration for the ArduPilot autopilot software,
using the waf build system (https://waf.io/). It defines how ArduPilot is
configured, compiled, and deployed across multiple vehicle types and hardware platforms.

Build System Architecture:
--------------------------
ArduPilot uses waf (a Python-based build system) with custom extensions located
in Tools/ardupilotwaf/. The build process supports:
  - Multiple vehicle types (Copter, Plane, Rover, Sub, Blimp, AntennaTracker)
  - 150+ hardware boards (ChibiOS/ARM, Linux, ESP32, SITL simulation)
  - Cross-compilation for ARM Cortex-M and other embedded targets
  - Modular feature flag system for capability selection
  - Bootloader and peripheral firmware builds

Relationship to Tools/ardupilotwaf:
-----------------------------------
This wscript file imports and uses modules from Tools/ardupilotwaf/ including:
  - ardupilotwaf.py: Core waf extensions and build helpers
  - boards.py: Board definition system and hardware configurations
  - build_options.py: Feature flags for enabling/disabling capabilities

Key Build Phases:
-----------------
1. init(): Called before option parsing, sets up build variant based on board
2. options(): Defines all command-line options (--board, --debug, feature flags, etc.)
3. configure(): Detects toolchain, validates board, configures build environment
4. build(): Compiles source files, links binaries, generates dynamic sources (MAVLink, DroneCAN)

Common Usage:
-------------
  Configure for a board:    ./waf configure --board=Pixhawk1
  Build all vehicles:       ./waf copter plane rover sub
  Build specific vehicle:   ./waf copter
  Upload to board:          ./waf copter --upload
  Run tests:                ./waf check
  Clean build:              ./waf clean
  List available boards:    ./waf list_boards

For complete documentation, see: https://ardupilot.org/dev/docs/building-the-code.html
"""

import optparse
import os.path
import os
import sys
import subprocess
import json
import fnmatch
sys.path.insert(0, 'Tools/ardupilotwaf/')
sys.path.insert(0, 'Tools/scripts/')

import ardupilotwaf
import boards
import shutil
import build_options
import glob

from waflib import Build, ConfigSet, Configure, Context, Utils
from waflib.Configure import conf

# TODO: implement a command 'waf help' that shows the basic tasks a
# developer might want to do: e.g. how to configure a board, compile a
# vehicle, compile all the examples, add a new example. Should fit in
# less than a terminal screen, ideally commands should be copy
# pastable. Add the 'export waf="$PWD/waf"' trick to be copy-pastable
# as well.

# TODO: replace defines with the use of the generated ap_config.h file
# this makes recompilation at least when defines change. which might
# be sufficient.

# Default installation prefix for Linux boards
default_prefix = '/usr/'

# Override Build execute and Configure post_recurse methods for autoconfigure purposes
Build.BuildContext.execute = ardupilotwaf.ap_autoconfigure(Build.BuildContext.execute)
Configure.ConfigurationContext.post_recurse = ardupilotwaf.ap_configure_post_recurse()


# Get the GitHub Actions summary file path
is_ci = os.getenv('CI')


def _set_build_context_variant(board):
    for c in Context.classes:
        if not issubclass(c, Build.BuildContext):
            continue
        c.variant = board

# Remove all submodules and then sync
@conf
def submodule_force_clean(ctx):
    whitelist = {
                            'COLCON_IGNORE',
                            'esp_idf',
                          }

    # Get all items in the modules folder
    module_list = os.scandir('modules')

    # Delete all directories except those in the whitelist
    for module in module_list:
        if (module.is_dir()) and (module.name not in whitelist):
            shutil.rmtree(module)

    submodulesync(ctx)

# run Tools/gittools/submodule-sync.sh to sync submodules
@conf
def submodulesync(ctx):
    subprocess.call(['Tools/gittools/submodule-sync.sh'])

def init(ctx):
    """
    Initialize build context before option parsing.
    
    This function is called by waf before command-line options are parsed.
    It performs early initialization tasks:
    
    1. Board Variant Selection:
       - Loads cached board selection from previous configure
       - Sets up build variant directory structure (e.g., build/sitl, build/Pixhawk1)
       - Each board gets its own build directory to allow parallel builds
    
    2. Configuration Cache Management:
       - Attempts to load previous configuration from build cache
       - Determines if autoconfiguration should run (--no-autoconfig disables)
       - Autoconfig automatically reconfigures when build files change
    
    3. VS Code Integration:
       - Generates tasklist.json for VS Code task provider extension
       - Allows VS Code to discover available build targets dynamically
       - Updates when boards or vehicle types change
    
    The board selection priority is:
       1. Command-line option (--board=BOARDNAME)
       2. Previously configured board from cache
       3. Default to 'sitl' if neither is available
    
    @param ctx: Waf context object containing options and environment
    """
    # Generate Task List, so that VS Code extension can keep track
    # of changes to possible build targets
    generate_tasklist(ctx, False)
    
    # Load configuration from previous build if available
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except EnvironmentError:
        # No cached configuration found, will use defaults or command-line options
        return

    # Set autoconfiguration behavior: 'clobber' forces reconfigure, False disables
    Configure.autoconfig = 'clobber' if env.AUTOCONFIG else False

    # Determine board to build for: command-line takes precedence over cached value
    board = ctx.options.board or env.BOARD

    if not board:
        return

    # Define the variant build commands according to the board
    # This sets up separate build directories for each board (e.g., build/sitl/)
    _set_build_context_variant(board)

def add_build_options(g):
    """
    Add feature flag options from Tools/scripts/build_options.py to command-line parser.
    
    ArduPilot uses a comprehensive feature flag system to enable/disable capabilities
    at compile time. This allows customization of firmware to:
      - Reduce binary size for resource-constrained boards
      - Disable unused features to save flash/RAM
      - Enable experimental or board-specific features
      - Configure peripheral support (cameras, gimbals, etc.)
    
    Feature Flag System:
    --------------------
    Each option in build_options.BUILD_OPTIONS defines a feature with:
      - config_option(): Name of the feature (e.g., "ENABLE_SCRIPTING")
      - description: Human-readable description
      - define: C++ preprocessor macro to define (e.g., "AP_SCRIPTING_ENABLED")
      - default: Whether enabled by default for various board classes
    
    For each feature, this function creates:
      1. --enable-FEATURE flag (enables the feature)
      2. --disable-FEATURE flag (disables the feature)
      3. Lowercase aliases with dashes for compatibility
    
    Impact on Binary Size:
    ----------------------
    Disabling features can significantly reduce flash usage:
      - Disabling scripting: ~100KB flash savings
      - Disabling vision/precision landing: ~50KB
      - Disabling mount support: ~20KB
    
    Usage Examples:
    ---------------
      Enable scripting:  ./waf configure --board=Pixhawk1 --enable-scripting
      Disable camera:    ./waf configure --board=Pixhawk1 --disable-camera
      Multiple flags:    ./waf configure --board=Pixhawk1 --enable-scripting --disable-mount
    
    See Tools/scripts/build_options.py for complete list of available features.
    
    @param g: Argument group for build configuration options
    """
    # Iterate through all defined build options from build_options.py
    for opt in build_options.BUILD_OPTIONS:
        # Generate --enable-FEATURE option name
        enable_option = "--" + opt.config_option()
        # Generate --disable-FEATURE option name
        disable_option = enable_option.replace("--enable", "--disable")
        
        # Prepare user-friendly descriptions
        enable_description = opt.description
        if not enable_description.lower().startswith("enable"):
            enable_description = "Enable " + enable_description
        disable_description = "Disable " + enable_description[len("Enable "):]
        
        # Add --enable-FEATURE option
        g.add_option(enable_option,
                     action='store_true',
                     default=False,
                     help=enable_description)
        # Add --disable-FEATURE option
        g.add_option(disable_option,
                     action='store_true',
                     default=False,
                     help=disable_description)

        # Also add entirely-lower-case equivalents with underscores replaced with dashes
        # This provides compatibility for different command-line conventions
        # Example: --enable-scripting and --enable_scripting both work
        lower_enable_option = enable_option.lower().replace("_", "-")
        if lower_enable_option != enable_option:
            g.add_option(lower_enable_option,
                         action='store_true',
                         default=False,
                         help=optparse.SUPPRESS_HELP)  # Hidden from help output
        lower_disable_option = disable_option.lower().replace("_", "-")
        if lower_disable_option != disable_option:
            g.add_option(lower_disable_option,
                         action='store_true',
                         default=False,
                         help=optparse.SUPPRESS_HELP)  # Hidden from help output

def add_script_options(g):
    """
    Add options for embedding Lua scripts into firmware ROMFS.
    
    ArduPilot supports onboard Lua scripting for custom behaviors and drivers.
    Scripts can be loaded from SD card or embedded directly in firmware ROMFS.
    
    This function scans for available Lua drivers and applets in:
      - libraries/AP_Scripting/drivers/  - Sensor/peripheral drivers written in Lua
      - libraries/AP_Scripting/applets/  - Application scripts for custom behaviors
    
    For each script found, creates a command-line option to embed it:
      --embed-SCRIPTNAME: Embeds the script in firmware ROMFS
    
    Benefits of Embedding Scripts:
      - No SD card required for critical functionality
      - Scripts available immediately on boot
      - Protected from accidental deletion
      - Useful for custom board configurations
    
    Example:
      ./waf configure --board=Pixhawk1 --embed-dronecan_battery_monitor
    
    @param g: Argument group for configuration options
    """
    # Scan for available Lua driver scripts
    driver_list = glob.glob(os.path.join(Context.run_dir, "libraries/AP_Scripting/drivers/*.lua"))
    # Scan for available Lua applet scripts
    applet_list = glob.glob(os.path.join(Context.run_dir, "libraries/AP_Scripting/applets/*.lua"))
    
    # Create --embed option for each script found
    for d in driver_list + applet_list:
        bname = os.path.basename(d)
        embed_name = bname[:-4]  # Remove .lua extension
        embed_option = "--embed-%s" % embed_name
        g.add_option(embed_option,
                     action='store_true',
                     default=False,
                     help="Embed %s in ROMFS" % bname)

def options(opt):
    """
    Define all command-line options for ArduPilot build system.
    
    This function is called by waf to populate the command-line parser with
    all available build options. Options are organized into groups:
    
    Configure Group Options:
    ------------------------
    Core build configuration options including:
      --board: Target hardware board (Pixhawk1, Linux, SITL, etc.)
      --debug: Enable debug build with reduced optimization
      --toolchain: Override default cross-compiler toolchain
      --bootloader: Build bootloader instead of main firmware
    
    Feature Flags:
    --------------
    Enable/disable capabilities via --enable-*/--disable-* flags.
    Added automatically from Tools/scripts/build_options.py
    
    Linux-Specific Options:
    -----------------------
    Installation paths, rsync deployment, benchmarks, hardware-specific features
    
    Development Options:
    --------------------
    Testing, debugging, profiling, and developer convenience options
    
    Option Naming Conventions:
    --------------------------
    - Boolean flags use --enable-*/--disable-* or --OPTION/--no-OPTION patterns
    - Value options use --OPTION=VALUE or --OPTION VALUE
    - Underscores and dashes are interchangeable in flag names
    
    @param opt: Waf options context for command-line parsing
    """
    # Load waf tools for C/C++ compilation, unit testing, and Python support
    opt.load('compiler_cxx compiler_c waf_unit_test python')
    # Load ArduPilot-specific waf extensions
    opt.load('ardupilotwaf')
    # Load build summary reporting tool
    opt.load('build_summary')

    # Get the configuration option group (organizes related options together)
    g = opt.ap_groups['configure']

    # Board selection option - most critical configuration choice
    # Determines target hardware, toolchain, and available features
    boards_names = boards.get_boards_names()
    removed_names = boards.get_removed_boards()
    g.add_option('--board',
        action='store',
        default=None,
        help='Target board to build, choices are %s.' % ', '.join(boards_names))

    # Debug build configuration options
    # Debug builds use -O0 optimization and enable additional runtime checks
    g.add_option('--debug',
        action='store_true',
        default=False,
        help='Configure as debug variant.')

    # Debug symbols can be added to release builds for profiling/debugging
    # without the performance impact of full debug mode (-O0)
    g.add_option('--debug-symbols', '-g',
        action='store_true',
        default=False,
        help='Add debug symbolds to build.')

    # VS Code integration for debugging embedded targets
    # Generates .vscode/launch.json with OpenOCD configuration
    g.add_option('--vs-launch',
        action='store_true',
        default=False,
        help='Generate wscript environment variable to .vscode/setting.json for Visual Studio Code')
    
    # Watchdog timer can be disabled for debugging (prevents resets during breakpoints)
    g.add_option('--disable-watchdog',
        action='store_true',
        default=False,
        help='Build with watchdog disabled.')

    # Code coverage analysis for testing (adds instrumentation)
    g.add_option('--coverage',
                 action='store_true',
                 default=False,
                 help='Configure coverage flags.')

    # Compiler warning configuration
    # -Werror treats all warnings as errors (enforced in CI builds)
    g.add_option('--Werror',
        action='store_true',
        default=None,
        help='build with -Werror.')

    g.add_option('--disable-Werror',
        action='store_true',
        default=None,
        help='Disable -Werror.')
    
    # Toolchain override for cross-compilation
    # Default toolchain is selected based on board (e.g., arm-none-eabi-gcc for ARM)
    # Use "native" to compile for host system (useful for tools)
    g.add_option('--toolchain',
        action='store',
        default=None,
        help='Override default toolchain used for the board. Use "native" for using the host toolchain.')

    g.add_option('--disable-gccdeps',
        action='store_true',
        default=False,
        help='Disable the use of GCC dependencies output method and use waf default method.')

    g.add_option('--enable-asserts',
        action='store_true',
        default=False,
        help='enable OS level asserts.')

    g.add_option('--save-temps',
        action='store_true',
        default=False,
        help='save compiler temporary files.')
    
    g.add_option('--enable-malloc-guard',
        action='store_true',
        default=False,
        help='enable malloc guard regions.')

    g.add_option('--enable-stats',
        action='store_true',
        default=False,
        help='enable OS level thread statistics.')

    # Bootloader build mode
    # ArduPilot boards use a custom bootloader for firmware updates and recovery
    # Bootloader runs first on boot and can load main firmware or enter DFU mode
    g.add_option('--bootloader',
        action='store_true',
        default=False,
        help='Configure for building a bootloader.')

    # Cryptographic firmware signing for secure boot
    # Requires matching public key in bootloader to verify firmware authenticity
    # Prevents loading of unsigned or modified firmware
    g.add_option('--signed-fw',
        action='store_true',
        default=False,
        help='Configure for signed firmware support.')

    # Path to RSA private key for firmware signing
    # Public key must be compiled into bootloader
    g.add_option('--private-key',
                 action='store',
                 default=None,
            help='path to private key for signing firmware.')
    
    # Autoconfiguration automatically reruns configure when build files change
    # Detects changes to wscript, board definitions, or build options
    # Disable for reproducible builds or when working with specific configurations
    g.add_option('--no-autoconfig',
        dest='autoconfig',
        action='store_false',
        default=True,
        help='''Disable autoconfiguration feature. By default, the build system
triggers a reconfiguration whenever it thinks it's necessary - this
option disables that.
''')

    g.add_option('--no-submodule-update',
        dest='submodule_update',
        action='store_false',
        default=True,
        help='''Don't update git submodules. Useful for building with
submodules at specific revisions.
''')

    g.add_option('--enable-header-checks', action='store_true',
        default=False,
        help="Enable checking of headers")

    g.add_option('--default-parameters',
        default=None,
        help='set default parameters to embed in the firmware')

    g.add_option('--enable-math-check-indexes',
                 action='store_true',
                 default=False,
                 help="Enable checking of math indexes")

    g.add_option('--disable-scripting', action='store_true',
                 default=False,
                 help="Disable onboard scripting engine")

    g.add_option('--enable-scripting', action='store_true',
                 default=False,
                 help="Enable onboard scripting engine")

    g.add_option('--no-gcs', action='store_true',
                 default=False,
                 help="Disable GCS code")
    
    g.add_option('--scripting-checks', action='store_true',
                 default=True,
                 help="Enable runtime scripting sanity checks")

    g.add_option('--enable-onvif', action='store_true',
                 default=False,
                 help="Enables and sets up ONVIF camera control")

    g.add_option('--scripting-docs', action='store_true',
                 default=False,
                 help="enable generation of scripting documentation")

    g.add_option('--enable-opendroneid', action='store_true',
                 default=False,
                 help="Enables OpenDroneID")

    g.add_option('--enable-check-firmware', action='store_true',
                 default=False,
                 help="Enables firmware ID checking on boot")

    g.add_option('--enable-custom-controller', action='store_true',
                 default=False,
                 help="Enables custom controller")

    g.add_option('--enable-gps-logging', action='store_true',
                 default=False,
                 help="Enables GPS logging")

    g.add_option('--disable-networking', action='store_true',
                 help="Disable the networking API code")

    g.add_option('--enable-networking-tests', action='store_true',
                 help="Enable the networking test code. Automatically enables networking.")
    
    g.add_option('--enable-dronecan-tests', action='store_true',
                 default=False,
                 help="Enables DroneCAN tests in sitl")

    g.add_option('--sitl-littlefs', action='store_true',
                 default=False,
                 help="Enable littlefs for filesystem access on SITL (under construction)")

    g = opt.ap_groups['linux']

    linux_options = ('--prefix', '--destdir', '--bindir', '--libdir')
    for k in linux_options:
        option = opt.parser.get_option(k)
        if option:
            opt.parser.remove_option(k)
            g.add_option(option)

    g.add_option('--apstatedir',
        action='store',
        default='',
        help='''Where to save data like parameters, log and terrain.
This is the --localstatedir + ArduPilots subdirectory [default:
board-dependent, usually /var/lib/ardupilot]''')

    g.add_option('--rsync-dest',
        dest='rsync_dest',
        action='store',
        default='',
        help='''Destination for the rsync Waf command. It can be passed during
configuration in order to save typing.
''')

    g.add_option('--enable-benchmarks',
        action='store_true',
        default=False,
        help='Enable benchmarks.')

    g.add_option('--enable-lttng', action='store_true',
        default=False,
        help="Enable lttng integration")

    g.add_option('--disable-libiio', action='store_true',
        default=False,
        help="Don't use libiio even if supported by board and dependencies available")

    g.add_option('--disable-tests', action='store_true',
        default=False,
        help="Disable compilation and test execution")

    g.add_option('--enable-sfml', action='store_true',
                 default=False,
                 help="Enable SFML graphics library")

    g.add_option('--enable-sfml-joystick', action='store_true',
                 default=False,
                 help="Enable SFML joystick input library")

    g.add_option('--enable-sfml-audio', action='store_true',
                 default=False,
                 help="Enable SFML audio library")

    g.add_option('--osd', action='store_true',
                 default=False,
                 help="Enable OSD support")

    g.add_option('--osd-fonts', action='store_true',
                 default=False,
                 help="Enable OSD support with fonts")
    
    g.add_option('--sitl-osd', action='store_true',
                 default=False,
                 help="Enable SITL OSD")

    g.add_option('--sitl-rgbled', action='store_true',
                 default=False,
                 help="Enable SITL RGBLed")

    g.add_option('--force-32bit', action='store_true',
                 default=False,
                 help="Force 32bit build")

    g.add_option('--build-dates', action='store_true',
                 default=False,
                 help="Include build date in binaries.  Appears in AUTOPILOT_VERSION.os_sw_version")

    g.add_option('--sitl-flash-storage',
        action='store_true',
        default=False,
        help='Use flash storage emulation.')

    g.add_option('--ekf-double',
        action='store_true',
        default=False,
        help='Configure EKF as double precision.')

    g.add_option('--ekf-single',
        action='store_true',
        default=False,
        help='Configure EKF as single precision.')
    
    g.add_option('--static',
        action='store_true',
        default=False,
        help='Force a static build')

    g.add_option('--postype-single',
        action='store_true',
        default=False,
        help='force single precision postype_t')

    g.add_option('--consistent-builds',
        action='store_true',
        default=False,
        help='force consistent build outputs for things like __LINE__')

    g.add_option('--extra-hwdef',
	    action='store',
	    default=None,
	    help='Extra hwdef.dat file for custom build.')

    g.add_option('--assert-cc-version',
                 default=None,
                 help='fail configure if not using the specified gcc version')

    g.add_option('--num-aux-imus',
                 type='int',
                 default=0,
                 help='number of auxiliary IMUs')

    g.add_option('--board-start-time',
                 type='int',
                 default=0,
                 help='zero time on boot in microseconds')

    g.add_option('--enable-iomcu-profiled-support',
                    action='store_true',
                    default=False,
                    help='enable iomcu profiled support')

    g.add_option('--enable-new-checking',
        action='store_true',
        default=False,
        help='enables checking of new to ensure NEW_NOTHROW is used')

    # support enabling any option in build_options.py
    add_build_options(g)

    # support embedding lua drivers and applets
    add_script_options(g)
    
def _collect_autoconfig_files(cfg):
    for m in sys.modules.values():
        paths = []
        if hasattr(m, '__file__') and m.__file__ is not None:
            paths.append(m.__file__)
        elif hasattr(m, '__path__'):
            for p in m.__path__:
                if p is not None:
                    paths.append(p)

        for p in paths:
            if p in cfg.files or not os.path.isfile(p):
                continue

            with open(p, 'rb') as f:
                cfg.hash = Utils.h_list((cfg.hash, f.read()))
                cfg.files.append(p)

def configure(cfg):
    """
    Configure build environment for selected board and options.
    
    This is the main configuration phase called when running './waf configure'.
    It performs comprehensive build environment setup:
    
    Configuration Steps:
    --------------------
    1. Board Selection and Validation:
       - Validates board name against available boards
       - Case-insensitive board name matching
       - Defaults to 'sitl' if no board specified
    
    2. Toolchain Detection:
       - Detects C/C++ compiler for target platform
       - Validates cross-compiler for ARM/embedded targets
       - Checks compiler version compatibility
       - Configures compiler flags (optimization, warnings, etc.)
    
    3. Build Variant Setup:
       - Creates board-specific build directory (build/BOARD/)
       - Configures separate environments for each board
       - Allows parallel builds for different boards
    
    4. Feature Configuration:
       - Processes --enable-*/--disable-* feature flags
       - Applies board-specific feature defaults
       - Generates preprocessor defines (AP_*_ENABLED macros)
       - Validates feature dependencies
    
    5. Dependency Detection:
       - Checks for required libraries (pthread, rt, etc.)
       - Detects optional dependencies (gbenchmark, gtest)
       - Configures submodule requirements (MAVLink, DroneCAN)
    
    6. Header Generation:
       - Generates ap_config.h with all feature defines
       - Creates version headers (ap_version.h)
       - Prepares hwdef-generated headers for ChibiOS boards
    
    7. Board-Specific Configuration:
       - Calls board.configure() for platform-specific setup
       - Configures memory layout, peripheral mapping
       - Sets up bootloader parameters
    
    After configuration, build environment is cached for subsequent builds.
    Reconfigure automatically when build files change (unless --no-autoconfig).
    
    @param cfg: Waf configuration context
    """
    # GitHub Actions integration: group configuration output
    if is_ci:
        print(f"::group::Waf Configure")
    
    # Default to SITL (Software In The Loop) simulation if no board specified
    # SITL allows testing without hardware
    if cfg.options.board is None:
        cfg.options.board = 'sitl'

    # Validate board name with case-insensitive matching
    # Allows --board=pixhawk1 or --board=PIXHAWK1
    boards_names = boards.get_boards_names()
    if not cfg.options.board in boards_names:
        for b in boards_names:
            if b.upper() == cfg.options.board.upper():
                cfg.options.board = b
                break
        
    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug
    cfg.env.DEBUG_SYMBOLS = cfg.options.debug_symbols
    cfg.env.COVERAGE = cfg.options.coverage
    cfg.env.AUTOCONFIG = cfg.options.autoconfig

    _set_build_context_variant(cfg.env.BOARD)
    cfg.setenv(cfg.env.BOARD)

    if cfg.options.signed_fw:
        cfg.env.AP_SIGNED_FIRMWARE = True
        cfg.options.enable_check_firmware = True

    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug
    cfg.env.VS_LAUNCH = cfg.options.vs_launch
    cfg.env.DEBUG_SYMBOLS = cfg.options.debug_symbols
    cfg.env.COVERAGE = cfg.options.coverage
    cfg.env.FORCE32BIT = cfg.options.force_32bit
    cfg.env.ENABLE_ASSERTS = cfg.options.enable_asserts
    cfg.env.BOOTLOADER = cfg.options.bootloader
    cfg.env.ENABLE_MALLOC_GUARD = cfg.options.enable_malloc_guard
    cfg.env.ENABLE_STATS = cfg.options.enable_stats
    cfg.env.SAVE_TEMPS = cfg.options.save_temps

    extra_hwdef = cfg.options.extra_hwdef
    if extra_hwdef is not None and not os.path.exists(extra_hwdef):
        raise FileNotFoundError(f"extra-hwdef file NOT found: '{cfg.options.extra_hwdef}'")
    cfg.env.HWDEF_EXTRA = cfg.options.extra_hwdef
    if cfg.env.HWDEF_EXTRA:
        cfg.env.HWDEF_EXTRA = os.path.abspath(cfg.env.HWDEF_EXTRA)

    cfg.env.OPTIONS = cfg.options.__dict__

    # Allow to differentiate our build from the make build
    cfg.define('WAF_BUILD', 1)

    cfg.msg('Autoconfiguration', 'enabled' if cfg.options.autoconfig else 'disabled')

    if cfg.options.static:
        cfg.msg('Using static linking', 'yes', color='YELLOW')
        cfg.env.STATIC_LINKING = True

    if cfg.options.num_aux_imus > 0:
        cfg.define('INS_AUX_INSTANCES', cfg.options.num_aux_imus)

    if cfg.options.board_start_time != 0:
        cfg.define('AP_BOARD_START_TIME', cfg.options.board_start_time)
        # also in env for hrt.c
        cfg.env.AP_BOARD_START_TIME = cfg.options.board_start_time

    # require python 3.8.x or later
    # also update `MIN_VER` in `./waf`
    cfg.load('python')
    cfg.check_python_version(minver=(3,8,0))

    cfg.load('ap_library')

    cfg.msg('Setting board to', cfg.options.board)
    cfg.get_board().configure(cfg)

    cfg.load('waf_unit_test')
    cfg.load('mavgen')
    cfg.load('dronecangen')

    cfg.env.SUBMODULE_UPDATE = cfg.options.submodule_update

    cfg.start_msg('Source is git repository')
    if cfg.srcnode.find_node('.git'):
        cfg.end_msg('yes')
    else:
        cfg.end_msg('no')
        cfg.env.SUBMODULE_UPDATE = False

    cfg.msg('Update submodules', 'yes' if cfg.env.SUBMODULE_UPDATE else 'no')
    cfg.load('git_submodule')

    if cfg.options.enable_benchmarks:
        cfg.load('gbenchmark')
    cfg.load('gtest')

    if cfg.env.BOARD == "sitl":
        cfg.start_msg('Littlefs')

        if cfg.options.sitl_littlefs:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

    cfg.load('littlefs')
    cfg.load('static_linking')
    cfg.load('build_summary')

    cfg.start_msg('Benchmarks')
    if cfg.env.HAS_GBENCHMARK:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Unit tests')
    if cfg.env.HAS_GTEST:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Scripting')
    if cfg.options.disable_scripting:
        cfg.end_msg('disabled', color='YELLOW')
    elif cfg.options.enable_scripting:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('maybe')
    cfg.recurse('libraries/AP_Scripting')

    cfg.recurse('libraries/AP_GPS')
    cfg.recurse('libraries/AP_HAL_SITL')
    cfg.recurse('libraries/SITL')

    cfg.recurse('libraries/AP_Networking')
    cfg.recurse('libraries/AP_DDS')

    cfg.start_msg('Scripting runtime checks')
    if cfg.options.scripting_checks:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Debug build')
    if cfg.env.DEBUG:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    if cfg.env.DEBUG:
        cfg.start_msg('VS Code launch')
        if cfg.env.VS_LAUNCH:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Coverage build')
    if cfg.env.COVERAGE:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Force 32-bit build')
    if cfg.env.FORCE32BIT:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.env.append_value('GIT_SUBMODULES', 'mavlink')

    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/libraries/',
    ])

    cfg.find_program('rsync', mandatory=False)
    if cfg.options.rsync_dest:
        cfg.msg('Setting rsync destination to', cfg.options.rsync_dest)
        cfg.env.RSYNC_DEST = cfg.options.rsync_dest

    if cfg.options.enable_header_checks:
        cfg.msg('Enabling header checks', cfg.options.enable_header_checks)
        cfg.env.ENABLE_HEADER_CHECKS = True
    else:
        cfg.env.ENABLE_HEADER_CHECKS = False

    # Always use system extensions
    cfg.define('_GNU_SOURCE', 1)

    if cfg.options.Werror:
        # print(cfg.options.Werror)
        if cfg.options.disable_Werror:
            cfg.options.Werror = False

    cfg.write_config_header(os.path.join(cfg.variant, 'ap_config.h'), guard='_AP_CONFIG_H_')

    # add in generated flags
    cfg.env.CXXFLAGS += ['-include', 'ap_config.h']

    cfg.remove_target_list()
    _collect_autoconfig_files(cfg)
    if is_ci:
        print("::endgroup::")

    if cfg.env.DEBUG and cfg.env.VS_LAUNCH:
        import vscode_helper
        vscode_helper.init_launch_json_if_not_exist(cfg)
        vscode_helper.update_openocd_cfg(cfg)

def collect_dirs_to_recurse(bld, globs, **kw):
    dirs = []
    globs = Utils.to_list(globs)

    if bld.bldnode.is_child_of(bld.srcnode):
        kw['excl'] = Utils.to_list(kw.get('excl', []))
        kw['excl'].append(bld.bldnode.path_from(bld.srcnode))

    for g in globs:
        for d in bld.srcnode.ant_glob(g + '/wscript', **kw):
            dirs.append(d.parent.relpath())
    return dirs

def list_boards(ctx):
    """List all available board names for configuration
    
    Prints space-separated list of all supported board names from Tools/ardupilotwaf/boards.py.
    Use with --board option during configure step.
    
    Usage: ./waf list_boards
    """
    print(*boards.get_boards_names())

def list_ap_periph_boards(ctx):
    """List all boards that support AP_Periph firmware
    
    AP_Periph is a CAN peripheral firmware for small CAN nodes (GPS, compass, etc.).
    These boards only support AP_Periph and bootloader builds, not full vehicle firmware.
    
    Usage: ./waf list_ap_periph_boards
    """
    print(*boards.get_ap_periph_boards())

@conf
def ap_periph_boards(ctx):
    return boards.get_ap_periph_boards()

# List of all vehicle types supported by ArduPilot
# Each vehicle has its own directory and wscript defining vehicle-specific build rules
# Vehicle binaries are named after the vehicle type (e.g., arducopter.elf, arduplane.apj)
vehicles = ['antennatracker', 'blimp', 'copter', 'heli', 'plane', 'rover', 'sub']

def generate_tasklist(ctx, do_print=True):
    """Generate tasklist.json file for VS Code ArduPilot extension integration
    
    This creates a JSON file mapping each board to its supported build targets,
    enabling IDE integration for build target selection and debugging.
    
    Args:
        ctx: waf context
        do_print: If True, print the generated task list to stdout
    
    Output:
        tasklist.json: Contains board configurations with available targets
            AP_Periph boards: Only support AP_Periph and bootloader
            IOMCU boards: Only support iofirmware and bootloader
            Linux/SITL boards: Support all vehicles (and replay for SITL)
            Other boards: Support all vehicles plus bootloader
    """
    boardlist = boards.get_boards_names()
    ap_periph_targets = boards.get_ap_periph_boards()
    tasks = []
    with open(os.path.join(Context.top_dir, "tasklist.json"), "w") as tlist:
        for board in boardlist:
            task = {}
            task['configure'] = board
            if board in ap_periph_targets:
                if 'sitl' not in board:
                    # we only support AP_Periph and bootloader builds
                    task['targets'] = ['AP_Periph', 'bootloader']
                else:
                    task['targets'] = ['AP_Periph']
            elif 'iofirmware' in board:
                task['targets'] = ['iofirmware', 'bootloader']
            else:
                if boards.is_board_based(board, boards.sitl):
                    task['targets'] = vehicles + ['replay']
                elif boards.is_board_based(board, boards.linux):
                    task['targets'] = vehicles
                else:
                    task['targets'] = vehicles + ['bootloader']
                    task['buildOptions'] = '--upload'
            tasks.append(task)
        tlist.write(json.dumps(tasks))
        if do_print:
            print(json.dumps(tasks))

def board(ctx):
    """Display the currently configured board name
    
    Reads the cached configuration and prints the board that was selected
    during the last 'waf configure' run. Useful for verifying current build configuration.
    
    Usage: ./waf board
    """
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except:
        print('No board currently configured')
        return

    print('Board configured to: {}'.format(env.BOARD))

def _build_cmd_tweaks(bld):
    if bld.cmd == 'check-all':
        bld.options.all_tests = True
        bld.cmd = 'check'

    if bld.cmd == 'check':
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.options.clear_failed_tests = True

def _build_dynamic_sources(bld):
    """
    Generate dynamic source files from definitions (MAVLink, DroneCAN, version headers).
    
    ArduPilot uses code generation for several subsystems to maintain consistency
    and support protocol evolution. This function sets up generation tasks for:
    
    MAVLink Protocol Headers:
    -------------------------
    MAVLink is the primary communication protocol between ArduPilot and ground stations.
    Headers are generated from XML message definitions in modules/mavlink/.
    
    Process:
      1. Parse modules/mavlink/message_definitions/v1.0/all.xml
      2. Generate C headers for all messages, enums, and commands
      3. Output to libraries/GCS_MAVLink/include/mavlink/v2.0/
      4. Supports MAVLink 2.0 protocol with signing and extensions
    
    Skipped for bootloader builds (bootloader doesn't use MAVLink).
    
    DroneCAN (UAVCAN) Protocol Headers:
    -----------------------------------
    DroneCAN is a CAN bus protocol for peripherals (GPS, compass, ESCs, etc.).
    Headers are generated from DSDL (Data Structure Description Language) files.
    
    Process:
      1. Parse DSDL files from modules/DroneCAN/DSDL/ and libraries/AP_DroneCAN/dsdl/
      2. Generate C headers with serialization/deserialization code
      3. Output to modules/DroneCAN/libcanard/dsdlc_generated/
      4. Different message sets for main firmware vs AP_Periph
    
    Generated only if board supports CAN (with_can flag).
    
    Version Header Generation:
    --------------------------
    ap_version.h contains git commit hash, version string, and build metadata.
    Regenerated on every build to track firmware version.
    
    @param bld: Waf build context
    """
    # Generate MAVLink protocol headers (not needed for bootloader)
    if not bld.env.BOOTLOADER:
        bld(
            features='mavgen',
            source='modules/mavlink/message_definitions/v1.0/all.xml',
            output_dir='libraries/GCS_MAVLink/include/mavlink/v2.0/',
            name='mavlink',
            # Export include paths so all libraries can find MAVLink headers
            # TODO: mavgen tool should set this automatically
            export_includes=[
            bld.bldnode.make_node('libraries').abspath(),
            bld.bldnode.make_node('libraries/GCS_MAVLink').abspath(),
            ],
            )

    # Generate DroneCAN protocol headers for main firmware with full message set
    # Includes standard UAVCAN messages plus ArduPilot-specific extensions
    if (bld.get_board().with_can or bld.env.HAL_NUM_CAN_IFACES) and not bld.env.AP_PERIPH:
        bld(
            features='dronecangen',
            # Scan for DSDL directories (lowercase names are message namespaces)
            source=bld.srcnode.ant_glob('modules/DroneCAN/DSDL/[a-z]* libraries/AP_DroneCAN/dsdl/[a-z]*', dir=True, src=False),
            output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
            name='dronecan',
            export_includes=[
                bld.bldnode.make_node('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath(),
                bld.srcnode.find_dir('modules/DroneCAN/libcanard/').abspath(),
                bld.srcnode.find_dir('libraries/AP_DroneCAN/canard/').abspath(),
                ]
            )
    # Generate DroneCAN headers for AP_Periph (peripheral firmware)
    # Uses all message types since AP_Periph can implement various node types
    elif bld.env.AP_PERIPH:
        bld(
            features='dronecangen',
            # Include all DSDL directories (not filtered by case)
            source=bld.srcnode.ant_glob('modules/DroneCAN/DSDL/* libraries/AP_DroneCAN/dsdl/*', dir=True, src=False),
            output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
            name='dronecan',
            export_includes=[
                bld.bldnode.make_node('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath(),
                bld.srcnode.find_dir('modules/DroneCAN/libcanard/').abspath(),
            ]
        )

    # Generate DDS (ROS2) integration headers if enabled
    bld.recurse("libraries/AP_DDS")

    # Version header generation function
    # Extracts git version info and writes to ap_version.h
    def write_version_header(tsk):
        bld = tsk.generator.bld
        return bld.write_version_header(tsk.outputs[0].abspath())

    # Generate ap_version.h with git commit hash and version information
    # Regenerated on every build to track firmware versions
    # Contains: git hash, branch, version string, build date (if enabled)
    bld(
        name='ap_version',
        target='ap_version.h',
        vars=['AP_VERSION_ITEMS'],
        rule=write_version_header,
    )

    # Add build directory to include path for generated headers
    # Allows #include "ap_version.h" and #include "ap_config.h"
    bld.env.prepend_value('INCLUDES', [
        bld.bldnode.abspath(),
    ])

def _build_common_taskgens(bld):
    """
    Create common build tasks shared across all vehicles and tools.
    
    This function sets up shared libraries and testing frameworks used
    by multiple build targets.
    
    ArduPilot Libraries Static Library:
    -----------------------------------
    The 'ap' static library contains all ArduPilot libraries compiled with
    vehicle type set to UNKNOWN. This allows tools and examples to link
    against the libraries without vehicle-specific code.
    
    NOTE: This is a transitional architecture. Currently, each vehicle
    (copter, plane, rover, etc.) also builds its own copy of the libraries
    with vehicle-specific defines. Future work may reduce vehicle dependencies
    and unify library builds.
    
    Testing Framework Libraries:
    ----------------------------
    GTest and Google Benchmark are optional dependencies for unit tests
    and performance benchmarks. Built only if detected during configuration.
    
    @param bld: Waf build context
    """
    # Build static library containing all ArduPilot libraries
    # Used by tools (AP_Periph, examples, utilities) that don't need vehicle code
    # Vehicle type set to UNKNOWN to avoid vehicle-specific compilation
    bld.ap_stlib(
        name='ap',
        ap_vehicle='UNKNOWN',
        ap_libraries=bld.ap_get_all_libraries(),
    )

    # Build GTest library if available (for unit tests)
    # Includes ap_config.h to match feature configuration of tested code
    if bld.env.HAS_GTEST:
        bld.libgtest(cxxflags=['-include', 'ap_config.h'])

    # Build Google Benchmark library if available (for performance tests)
    if bld.env.HAS_GBENCHMARK:
        bld.libbenchmark()

def _build_recursion(bld):
    """
    Recursively discover and build all source directories.
    
    ArduPilot has a distributed build structure where each component
    (vehicles, libraries, tools, examples, tests) has its own wscript file
    defining local build tasks. This function discovers and recurses into
    all relevant directories.
    
    Directory Categories:
    ---------------------
    1. Vehicles: ArduCopter, ArduPlane, Rover, ArduSub, Blimp, AntennaTracker
       - Each vehicle builds a binary (e.g., arducopter.elf)
       - Links against libraries with vehicle-specific defines
    
    2. Libraries: libraries/AP_*, libraries/AC_*, etc.
       - Core functionality organized into reusable modules
       - Each library may have examples, tests, benchmarks subdirectories
    
    3. Tools: Tools/Replay, Tools/AP_Periph, etc.
       - Utility programs for development and testing
       - Log replay, firmware signing, parameter checking, etc.
    
    4. HAL Implementations: libraries/AP_HAL_*
       - Platform-specific hardware abstraction implementations
       - Processed separately due to special handling
    
    5. Tests and Benchmarks: libraries/*/tests, libraries/*/benchmarks
       - Unit tests using GTest framework
       - Performance benchmarks using Google Benchmark
    
    Special Firmware Builds:
    ------------------------
    - IOMCU: I/O coprocessor firmware for some Pixhawk boards
    - AP_Periph: CAN peripheral firmware for custom CAN nodes
    - Scripting: Lua scripting engine (conditionally included)
    - ONVIF: Camera control protocol (conditionally included)
    
    Directory Discovery:
    --------------------
    Directories are discovered by scanning for wscript files matching patterns.
    Patterns use ant-style globs (* for any directory level).
    
    Sorting Requirement:
    --------------------
    Directories must be processed in consistent order to ensure:
      - Repeated source files get same index in object file list
      - Reproducible builds (same input -> same output)
      - Avoid unnecessary recompilation from filesystem ordering differences
    
    @param bld: Waf build context
    """
    # Common directory patterns to search for wscript files
    # TODO: Each vehicle currently builds its own library copy
    # This should be unified to reduce build times
    common_dirs_patterns = [
        '*',                              # Top-level directories (vehicles)
        'Tools/*',                        # Development tools
        'libraries/*/examples/*',         # Library usage examples
        'libraries/*/tests',              # Library unit tests
        'libraries/*/utility/tests',      # Utility class tests
        'libraries/*/benchmarks',         # Performance benchmarks
    ]

    # Exclude directories that shouldn't be recursed
    common_dirs_excl = [
        'modules',           # External submodules (have their own build systems)
        'libraries/AP_HAL_*', # HAL implementations (processed separately)
    ]

    # HAL-specific directory patterns (tests, examples, benchmarks)
    hal_dirs_patterns = [
        'libraries/%s/tests',
        'libraries/%s/*/tests',
        'libraries/%s/*/benchmarks',
        'libraries/%s/examples/*',
    ]

    # Collect all directories to recurse based on patterns
    dirs_to_recurse = collect_dirs_to_recurse(
        bld,
        common_dirs_patterns,
        excl=common_dirs_excl,
    )
    
    # Add IOMCU firmware if building for boards with I/O coprocessor
    if bld.env.IOMCU_FW is not None:
        if bld.env.IOMCU_FW:
            dirs_to_recurse.append('libraries/AP_IOMCU/iofirmware')

    # Add AP_Periph firmware if building peripheral firmware
    if bld.env.PERIPH_FW is not None:
        if bld.env.PERIPH_FW:
            dirs_to_recurse.append('Tools/AP_Periph')

    # Always include scripting library (conditionally compiled based on features)
    dirs_to_recurse.append('libraries/AP_Scripting')

    # Add ONVIF camera control if enabled
    if bld.env.ENABLE_ONVIF:
        dirs_to_recurse.append('libraries/AP_ONVIF')

    # Add HAL-specific directories for each HAL implementation in use
    for p in hal_dirs_patterns:
        dirs_to_recurse += collect_dirs_to_recurse(
            bld,
            [p % l for l in bld.env.AP_LIBRARIES],
        )

    # Sort directories to ensure consistent build order
    # This is critical for reproducible builds and avoiding unnecessary recompilation
    # Random filesystem ordering can cause the same source file to get different
    # indices in the object list, triggering spurious rebuilds
    dirs_to_recurse.sort()

    # Recurse into each directory to process its wscript
    for d in dirs_to_recurse:
        bld.recurse(d)

def _build_post_funs(bld):
    if bld.cmd == 'check':
        bld.add_post_fun(ardupilotwaf.test_summary)
    else:
        bld.build_summary_post_fun()

    if bld.env.SUBMODULE_UPDATE:
        bld.git_submodule_post_fun()

def _load_pre_build(bld):
    '''allow for a pre_build() function in build modules'''
    if bld.cmd == 'clean':
        return
    brd = bld.get_board()
    if getattr(brd, 'pre_build', None):
        brd.pre_build(bld)    

def build(bld):
    """
    Main build phase - compiles source files and links binaries.
    
    This function is called when running './waf' or './waf build'.
    It orchestrates the complete build process in multiple phases:
    
    Build Phases and Task Groups:
    -----------------------------
    Waf executes build tasks in groups, with dependencies between groups.
    Tasks within a group can run in parallel.
    
    1. git_submodules (if --no-submodule-update not set):
       - Updates git submodules (MAVLink, ChibiOS, DroneCAN, etc.)
       - Ensures external dependencies are current
       - Can be disabled for offline builds or specific revisions
    
    2. dynamic_sources:
       - Generates MAVLink headers from XML definitions
       - Generates DroneCAN message headers from DSDL
       - Creates ap_version.h with git version info
       - Generates hwdef headers for ChibiOS boards
    
    3. build:
       - Compiles all library source files (libraries/AP_*/*)
       - Compiles vehicle-specific code (ArduCopter, ArduPlane, etc.)
       - Compiles HAL implementation for target board
       - Links final binaries (e.g., arducopter.elf, arduplane.apj)
    
    Configuration Hash Management:
    ------------------------------
    ap_config.h contains all feature defines and is included in every compilation.
    Its hash is tracked to force recompilation when configuration changes.
    
    Library Dependency Management:
    ------------------------------
    All ArduPilot libraries automatically:
      - Include ap_config.h (feature flags)
      - Link against MAVLink headers
      - Link against DroneCAN headers (if board supports CAN)
      - Link against LittleFS (if configured)
    
    Board-Specific Build Steps:
    ---------------------------
    Each board class implements board.build() for platform-specific steps:
      - ChibiOS boards: process hwdef, configure memory regions, bootloader
      - Linux boards: configure installation paths, systemd services
      - SITL: configure simulation features, networking
    
    Parallel Compilation:
    ---------------------
    Waf automatically parallelizes compilation across CPU cores.
    Use -j N to limit parallelism (e.g., './waf -j4' for 4 jobs).
    
    @param bld: Waf build context containing environment and task management
    """
    # GitHub Actions integration: group build output
    if is_ci:
        print(f"::group::Waf Build")
    
    # Calculate hash of ap_config.h to trigger recompilation when config changes
    # This ensures all files are rebuilt if feature flags change
    config_hash = Utils.h_file(bld.bldnode.make_node('ap_config.h').abspath())
    bld.env.CCDEPS = config_hash
    bld.env.CXXDEPS = config_hash

    # Use lazy task posting for better build parallelism
    # Tasks are posted as their dependencies complete
    bld.post_mode = Build.POST_LAZY

    # Load ArduPilot-specific build extensions
    bld.load('ardupilotwaf')

    # Configure default flags for all ArduPilot library compilations
    # Every library object file will:
    #   - Include ap_config.h automatically (feature flags)
    #   - Link against MAVLink generated headers
    bld.env.AP_LIBRARIES_OBJECTS_KW.update(
        use=['mavlink'],
        cxxflags=['-include', 'ap_config.h'],
    )

    # Allow board to run custom pre-build steps
    # Example: ChibiOS boards generate hwdef headers here
    _load_pre_build(bld)

    # Add DroneCAN dependency if board supports CAN bus
    if bld.get_board().with_can:
        bld.env.AP_LIBRARIES_OBJECTS_KW['use'] += ['dronecan']

    # Add LittleFS filesystem dependency if configured
    if bld.get_board().with_littlefs:
        bld.env.AP_LIBRARIES_OBJECTS_KW['use'] += ['littlefs']
        bld.littlefs()

    # Handle special build commands (check, check-all)
    _build_cmd_tweaks(bld)

    # Phase 1: Update git submodules if enabled
    if bld.env.SUBMODULE_UPDATE:
        bld.add_group('git_submodules')
        for name in bld.env.GIT_SUBMODULES:
            bld.git_submodule(name)

    # Phase 2: Generate dynamic source files (MAVLink, DroneCAN, version headers)
    bld.add_group('dynamic_sources')
    _build_dynamic_sources(bld)

    # Phase 3: Main compilation and linking
    bld.add_group('build')
    bld.get_board().build(bld)  # Board-specific build tasks
    _build_common_taskgens(bld)  # Common libraries and test frameworks

    # Recurse into all source directories for compilation
    _build_recursion(bld)

    # Register post-build actions (test summary, build summary, submodule status)
    _build_post_funs(bld)
    if is_ci:
        def print_ci_endgroup(bld):
            print(f"::endgroup::")
        bld.add_post_fun(print_ci_endgroup)


    if bld.env.DEBUG and bld.env.VS_LAUNCH:
        import vscode_helper
        vscode_helper.update_settings(bld)

# Define custom build commands for testing
# These commands build all programs and run unit tests
ardupilotwaf.build_command('check',
    program_group_list='all',
    doc='builds all programs and run tests',
)
ardupilotwaf.build_command('check-all',
    program_group_list='all',
    doc='shortcut for `waf check --alltests`',
)

# Define build commands for each vehicle type
# Usage: ./waf copter          - builds ArduCopter
#        ./waf plane           - builds ArduPlane
#        ./waf bootloader      - builds bootloader for current board
#        ./waf AP_Periph       - builds AP_Periph CAN peripheral firmware
#        ./waf replay          - builds log replay tool (SITL only)
for name in (vehicles + ['bootloader','iofirmware','AP_Periph','replay']):
    ardupilotwaf.build_command(name,
        program_group_list=name,
        doc='builds %s programs' % name,
    )

# Define build commands for program groups
# Usage: ./waf all            - builds everything
#        ./waf bin            - builds main binaries only
#        ./waf examples       - builds library examples
#        ./waf tests          - builds unit tests
#        ./waf benchmarks     - builds performance benchmarks
for program_group in ('all', 'bin', 'tool', 'examples', 'tests', 'benchmarks'):
    ardupilotwaf.build_command(program_group,
        program_group_list=program_group,
        doc='builds all programs of %s group' % program_group,
    )

class LocalInstallContext(Build.InstallContext):
    """Custom waf install context for local installation staging
    
    Installs built binaries to build/BOARD/install/ directory instead of system paths.
    Useful for preparing files before deployment to target system without requiring
    root privileges or modifying the host system.
    
    Usage: ./waf localinstall
    
    Output location: build/<board_name>/install/
    """
    cmd = 'localinstall'

    def __init__(self, **kw):
        super(LocalInstallContext, self).__init__(**kw)
        # Set installation destination to variant-specific install directory
        self.local_destdir = os.path.join(self.variant_dir, 'install')

    def execute(self):
        # Temporarily override destdir to use local staging area
        old_destdir = self.options.destdir
        self.options.destdir = self.local_destdir
        r = super(LocalInstallContext, self).execute()
        self.options.destdir = old_destdir
        return r

class RsyncContext(LocalInstallContext):
    """Custom waf context for syncing built binaries to remote target system
    
    Performs localinstall followed by rsync to deploy binaries to target board
    (typically Linux-based boards like Navio, BBBmini, etc.). Destination can be
    specified via --rsync-dest option during configure or rsync command.
    
    Usage: ./waf rsync --rsync-dest=user@target:/path/to/install
    
    Process:
        1. Runs localinstall to stage binaries in build/BOARD/install/
        2. Uses rsync to copy staged files to target system
        3. Preserves permissions and uses incremental transfer
    
    Requirements:
        - rsync must be installed on host system
        - SSH access to target system (if remote)
        - --rsync-dest must be configured
    """
    cmd = 'rsync'

    def __init__(self, **kw):
        super(RsyncContext, self).__init__(**kw)
        # Add rsync task generation before installation
        self.add_pre_fun(RsyncContext.create_rsync_taskgen)

    def create_rsync_taskgen(self):
        """Create the rsync task for file transfer to target system"""
        # Verify rsync is available
        if 'RSYNC' not in self.env:
            self.fatal('rsync program seems not to be installed, can\'t continue')

        self.add_group()

        # Create rsync task with appropriate options
        # -a: archive mode (preserves permissions, timestamps, etc.)
        tg = self(
            name='rsync',
            rule='${RSYNC} -a ${RSYNC_SRC}/ ${RSYNC_DEST}',
            always=True,  # Always run, even if files haven't changed
        )

        tg.env.RSYNC_SRC = self.local_destdir
        if self.options.rsync_dest:
            self.env.RSYNC_DEST = self.options.rsync_dest

        # Verify destination is configured
        if 'RSYNC_DEST' not in tg.env:
            self.fatal('Destination for rsync not defined. Either pass --rsync-dest here or during configuration.')

        tg.post()
