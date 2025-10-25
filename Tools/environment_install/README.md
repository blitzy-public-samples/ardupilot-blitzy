# ArduPilot Development Environment Setup

![Build Status](https://img.shields.io/badge/setup-automated-green)
![Platforms](https://img.shields.io/badge/platforms-linux%20%7C%20windows%20%7C%20macos-blue)

## Table of Contents
- [Overview](#overview)
- [Supported Platforms](#supported-platforms)
- [Quick Start](#quick-start)
- [Platform-Specific Installation](#platform-specific-installation)
  - [Ubuntu/Debian](#ubuntudebian)
  - [Windows](#windows)
  - [macOS](#macos)
  - [Arch Linux](#arch-linux)
  - [Alpine Linux](#alpine-linux)
  - [openSUSE Tumbleweed](#opensuse-tumbleweed)
- [Components Installed](#components-installed)
- [Python Virtual Environments](#python-virtual-environments)
- [ARM Toolchains](#arm-toolchains)
- [ROS Integration](#ros-integration)
- [Environment Variables](#environment-variables)
- [Troubleshooting](#troubleshooting)
- [Advanced Configuration](#advanced-configuration)
- [Maintenance and Updates](#maintenance-and-updates)

## Overview

The ArduPilot environment setup scripts provide automated installation of all dependencies, toolchains, and development tools required for building, testing, and debugging ArduPilot firmware across multiple platforms. These scripts handle:

- **Build Tools**: Compilers, linkers, make systems, and build dependencies
- **Python Environment**: Python packages for SITL simulation, testing, and ground control
- **ARM Toolchains**: Cross-compilation toolchains for STM32 and ARM Linux targets
- **Development Tools**: Debuggers, profilers, code analysis tools
- **Ground Control Software**: MAVProxy and related utilities
- **Optional Components**: ROS integration, code coverage tools, graphical libraries

**Source Files**: `/Tools/environment_install/`

**Safety Note**: These scripts modify system packages and environment variables. Review script contents before execution, especially on production systems.

## Supported Platforms

The environment installation scripts support the following operating systems and versions:

### Linux Distributions

| Distribution | Versions Supported | Script | Notes |
|-------------|-------------------|--------|-------|
| **Ubuntu** | 20.04 (Focal), 22.04 (Jammy), 23.04 (Lunar), 23.10 (Mantic), 24.04 (Noble), 24.10 (Oracular), 25.04 (Plucky) | `install-prereqs-ubuntu.sh` | Primary development platform |
| **Debian** | Bullseye (11), Bookworm (12) | `install-prereqs-ubuntu.sh` | Uses Ubuntu script with version detection |
| **Linux Mint** | 20.x (Una/Uma/Ulyssa/Ulyana), 21.x (Vanessa/Vera/Victoria/Virginia), 22.x (Wilma/Xia) | `install-prereqs-ubuntu.sh` | Automatically maps to Ubuntu equivalents |
| **Elementary OS** | 6.x (Jolnir) | `install-prereqs-ubuntu.sh` | Maps to Ubuntu Focal |
| **Arch Linux** | Rolling release | `install-prereqs-arch.sh` | Full support with AUR packages |
| **Alpine Linux** | 3.x+ | `install-prereqs-alpine.sh` | Minimal installation (experimental) |
| **openSUSE** | Tumbleweed (rolling) | `install-prereqs-openSUSE-Tumbleweed.sh` | Full support with zypper |

### Other Platforms

| Platform | Versions | Script | Notes |
|---------|----------|--------|-------|
| **macOS** | 10.15+ (Catalina and later) | `install-prereqs-mac.sh` | Intel and Apple Silicon (M1/M2) |
| **Windows** | 10/11 | `install-prereqs-windows.ps1` | Via Cygwin environment |

**Important**: Ubuntu 18.04 (Bionic) and Debian 10 (Buster) have reached end-of-life and are no longer supported by ArduPilot development tools.

## Quick Start

### Linux (Ubuntu/Debian)
```bash
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### macOS
```bash
cd ardupilot
Tools/environment_install/install-prereqs-mac.sh -y
```

### Windows
```powershell
# Run as Administrator in PowerShell
cd ardupilot\Tools\environment_install
.\install-prereqs-windows.ps1
```

### Arch Linux
```bash
cd ardupilot
Tools/environment_install/install-prereqs-arch.sh -y
```

The `-y` flag assumes "yes" to all prompts for unattended installation. Omit for interactive mode.

## Platform-Specific Installation

### Ubuntu/Debian

**Script**: `install-prereqs-ubuntu.sh`  
**Source**: `/Tools/environment_install/install-prereqs-ubuntu.sh`

#### Features
- Automatic version detection for Ubuntu, Debian, Linux Mint, and Elementary OS
- Python virtual environment creation on recent distributions (Bookworm+, Lunar+)
- Optional STM32 ARM toolchain installation
- MAVProxy and development tool integration
- Automatic `dialout` group membership for serial port access

#### Usage

**Interactive Installation** (recommended for first-time setup):
```bash
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh
```

**Unattended Installation**:
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

**Quiet Mode** (minimal output):
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -yq
```

#### Command-Line Options

| Option | Description |
|--------|-------------|
| `-y` | Assume yes to all prompts (unattended mode) |
| `-q` | Quiet mode - suppress non-essential output |

#### Installation Steps

The Ubuntu script performs the following operations:

1. **System Validation**
   - Checks for root execution (exits if run as root)
   - Installs and uses `lsb-release` for version detection
   - Maps derivative distributions to Ubuntu equivalents

2. **Package Installation**
   - Updates apt package lists
   - Installs base development tools: `build-essential`, `ccache`, `git`, `make`, `wget`
   - Installs Python 3 and development headers
   - Installs SITL dependencies: graphics libraries (SFML/CSFML), Python scientific stack
   - Optionally installs code coverage tools (lcov, gcovr)

3. **Python Environment Setup**
   - Creates Python virtual environment on Debian Bookworm, Ubuntu Lunar and newer
   - Installs Python packages: `pymavlink`, `MAVProxy`, `dronecan`, `empy==3.3.4`
   - Adds graphical packages if not disabled: `matplotlib`, `scipy`, `opencv-python`

4. **Toolchain Installation** (if accepted)
   - Downloads GCC ARM None EABI 10-2020-q4-major from ArduPilot server
   - Installs to `/opt/gcc-arm-none-eabi-10-2020-q4-major/`
   - Configures ccache wrappers for ARM compilation

5. **Environment Configuration**
   - Adds user to `dialout` group for serial port access
   - Optionally adds toolchain paths to `.profile` or `.bashrc`
   - Optionally adds ArduPilot tools to PATH
   - Optionally enables bash completion for ArduPilot commands
   - Configures ccache PATH for build acceleration

6. **Git Submodule Update**
   - Initializes and updates all git submodules recursively

#### Version-Specific Behavior

**Ubuntu 20.04 Focal / Debian Bullseye**:
- Uses system Python packages
- SFML version 2.5
- Includes `python-is-python3` symlink

**Ubuntu 22.04 Jammy**:
- Uses system Python packages
- SFML version 2.5
- Enhanced security defaults

**Ubuntu 23.04+ / Debian Bookworm+**:
- Creates Python virtual environment at `~/venv-ardupilot`
- SFML version 2.5 (2.6 on Noble+)
- Python packages installed in venv
- Requires explicit venv activation or automatic sourcing

#### Environment Variables Modified

The script prompts before modifying shell configuration files:

```bash
# STM32 toolchain PATH (if installed)
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH

# ArduPilot tools PATH
export PATH=/path/to/ardupilot/Tools/autotest:$PATH

# Bash completion (added to ~/.bashrc)
source /path/to/ardupilot/Tools/completion/completion.bash

# CCache PATH
export PATH=/usr/lib/ccache:$PATH

# Python venv activation (Ubuntu 23.04+)
source $HOME/venv-ardupilot/bin/activate
```

#### Docker Support

The script detects Docker/container environments and modifies behavior:
- Creates `.ardupilot_env` instead of modifying `.profile`
- Automatically sources `.ardupilot_env` from `.bashrc`
- Skips interactive prompts if `AP_DOCKER_BUILD=1`

#### Skipping Components

Use environment variables to skip optional components:

```bash
# Skip STM32 toolchain prompt (don't install)
export DO_AP_STM_ENV=0

# Skip external Python packages (pygame, intelhex)
export SKIP_AP_EXT_ENV=1

# Skip graphical packages (matplotlib, wxPython, opencv)
export SKIP_AP_GRAPHIC_ENV=1

# Skip code coverage tools
export SKIP_AP_COV_ENV=1

# Skip bash completion setup
export SKIP_AP_COMPLETION_ENV=1

# Skip git submodule update
export SKIP_AP_GIT_CHECK=1
```

#### Packages Removed

The script removes packages that interfere with firmware uploading:
- `modemmanager` - Interferes with serial port communication
- `brltty` - Conflicts with CH340/CP210x USB-serial adapters

### Windows

**Scripts**: 
- `install-prereqs-windows.ps1` - Environment only
- `install-prereqs-windows-andAPMSource.ps1` - Environment + ArduPilot source

**Source**: `/Tools/environment_install/install-prereqs-windows.ps1`

#### Overview

Windows development requires Cygwin to provide a Unix-like environment. The PowerShell scripts automate installation of:
- Cygwin 64-bit with required packages
- Python 3.9 in Cygwin environment
- MAVProxy for Windows
- ARM GCC compiler for STM32 boards

#### Prerequisites

- Windows 10 or Windows 11
- PowerShell with Administrator privileges
- Internet connection for downloads
- Approximately 5GB free disk space

#### Usage

**Option 1: Environment Only** (use with existing ArduPilot source):
```powershell
# Open PowerShell as Administrator
cd \path\to\ardupilot\Tools\environment_install
.\install-prereqs-windows.ps1
```

**Option 2: Environment + Source** (fresh installation):
```powershell
# Open PowerShell as Administrator
cd \path\to\Tools\environment_install
.\install-prereqs-windows-andAPMSource.ps1
```

#### Installation Process

**Step 1: Downloads** (install-prereqs-windows.ps1)
1. MAVProxy installer (latest)
2. Cygwin setup executable (x86_64)
3. ARM GCC Compiler 10-2020-q4-major (Windows)

**Step 2: Cygwin Installation**
- Install location: `C:\cygwin64`
- Mirror: `http://cygwin.mirror.constant.com`
- Packages installed:
  - Build tools: `autoconf`, `automake`, `make`, `gawk`, `libtool`
  - Compilers: `gcc-g++` (7.4.0), `cygwin32-gcc-g++`
  - Development: `git`, `ccache`, `gdb`, `ddd`, `xterm`
  - Libraries: `libexpat-devel`, `libxml2-devel`, `libxslt-devel`
  - Python: `python39`, `python39-devel`, `python39-pip`, `python39-future`, `python39-lxml`
  - Utilities: `procps-ng`, `zip`

**Step 3: Python Packages**
Installed via pip in Cygwin:
```bash
python3.9 -m pip install empy==3.3.4 pyserial pymavlink intelhex dronecan pexpect
```

**Step 4: ARM Toolchain**
- Installer: Silent installation to default location
- Path automatically configured in Windows

**Step 5: MAVProxy**
- Silent installation of latest version
- Available in Windows PATH

**Step 6: ArduPilot Source** (install-prereqs-windows-andAPMSource.ps1 only)
- Copies `APM_install.sh` to Cygwin home directory
- Executes within Cygwin to clone ArduPilot repository
- Initializes git submodules
- Configures waf for SITL build

#### Post-Installation

After script completion:

1. **Launch Cygwin Terminal**:
   - Start menu: Cygwin64 Terminal
   - or run: `C:\cygwin64\bin\mintty.exe -`

2. **Verify Installation**:
```bash
# In Cygwin terminal
cd ~/ardupilot  # if using andAPMSource script
./waf configure --board sitl
./waf build --target bin/arducopter
```

3. **Run SITL**:
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

#### Known Limitations

- Native Windows paths not supported in Cygwin
- Performance slower than native Linux
- Some hardware debugging tools unavailable
- Cross-compilation to ARM Linux not supported on Windows

### macOS

**Script**: `install-prereqs-mac.sh`  
**Source**: `/Tools/environment_install/install-prereqs-mac.sh`

#### Features
- Support for both Intel (x86_64) and Apple Silicon (arm64) Macs
- Homebrew package manager integration
- Shell detection (bash vs zsh) for proper configuration
- Optional pyenv for Python version management
- Native ARM cross-compilation toolchain

#### Prerequisites

- macOS 10.15 (Catalina) or later
- Xcode Command Line Tools
- Internet connection
- Approximately 3GB free disk space

#### Usage

**Interactive Installation**:
```bash
cd ardupilot
Tools/environment_install/install-prereqs-mac.sh
```

**Unattended Installation**:
```bash
Tools/environment_install/install-prereqs-mac.sh -y
```

#### Installation Process

**Step 1: Homebrew Installation**
- Automatically installs Homebrew if not present
- Updates Homebrew package database
- Installs core utilities: `gawk`, `coreutils`, `wget`

**Step 2: Xcode Command Line Tools**
- Verifies Xcode CLI tools are installed
- Prompts for installation if missing

**Step 3: Python Environment**
The script offers two Python management options:

**Option A: System Python** (default):
- Uses system Python 3 installation
- Installs packages via `python3 -m pip`

**Option B: pyenv** (prompted):
- Installs pyenv version 2.3.12
- Installs Python 3.10.4 with framework support
- Sets Python 3.10.4 as global default
- Configures shell for automatic pyenv activation

**Step 4: Development Tools**
- Installs ccache for build acceleration
- Configures ccache libexec path

**Step 5: Python Packages**
Base packages:
```
setuptools, future, lxml, matplotlib, pymavlink, MAVProxy, pexpect, 
geocoder, flake8, junitparser, empy==3.3.4, dronecan
```

Optional packages (if not skipped):
- External: `intelhex`, `gnureadline`
- Graphical: `wxPython`, `billiard`

**Step 6: ARM Toolchain** (if accepted)
- Downloads GCC ARM None EABI 10-2020-q4-major for macOS
- Installs to `/opt/gcc-arm-none-eabi-10-2020-q4-major/`
- Creates ccache symlinks in `/usr/local/opt/ccache/libexec/`

**Step 7: Environment Configuration**
Modifies shell configuration (`.bash_profile` for bash, `.zshrc` for zsh):
```bash
# Pyenv configuration (if installed)
export PYENV_ROOT=$HOME/.pyenv
export PATH=$PYENV_ROOT/bin:$PATH
eval "$(pyenv init --path)"
eval "$(pyenv init -)"

# ARM toolchain PATH (if installed)
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH

# ArduPilot tools PATH
export PATH=/path/to/ardupilot/Tools/autotest:$PATH

# Shell completion
source /path/to/ardupilot/Tools/completion/completion.bash  # or completion.zsh

# CCache PATH
export PATH=/usr/local/opt/ccache/libexec:$PATH
```

**Step 8: Git Submodules**
- Updates all git submodules recursively

#### macOS-Specific Notes

**Homebrew Python Conflicts**:
The script removes Python framework symlinks in `/usr/local/bin` that may conflict with Homebrew installations. This prevents upgrade failures when Homebrew updates Python dependencies.

**Apple Silicon (M1/M2)**:
- Native ARM64 builds supported
- ARM cross-compiler runs via Rosetta 2 if needed
- Some Python packages may require compilation time

**Shell Detection**:
- Automatically detects bash vs zsh
- Uses appropriate completion script (`.bash` or `.zsh`)
- Configures correct profile file

#### Skip Options

```bash
# Skip STM32 toolchain installation
export DO_AP_STM_ENV=0

# Skip external packages (intelhex, gnureadline)
export SKIP_AP_EXT_ENV=1

# Skip graphical packages (wxPython, billiard)
export SKIP_AP_GRAPHIC_ENV=1

# Skip bash/zsh completion
export SKIP_AP_COMPLETION_ENV=1
```

### Arch Linux

**Script**: `install-prereqs-arch.sh`  
**Source**: `/Tools/environment_install/install-prereqs-arch.sh`

#### Features
- Rolling release support
- Python virtual environment (venv) with system-site-packages
- Checksum verification for downloaded toolchains
- Pacman package management integration

#### Usage

```bash
cd ardupilot
Tools/environment_install/install-prereqs-arch.sh -y
```

#### Installation Process

**Step 1: User Configuration**
- Adds user to `uucp` group (serial port access on Arch)

**Step 2: System Packages**
```bash
# Base development
base-devel ccache git gsfonts tk wget gcc

# SITL dependencies
python-pip python-setuptools python-wheel python-wxpython opencv 
python-numpy python-scipy

# Legacy 32-bit support (for some tools)
lib32-glibc zip zlib ncurses
```

**Step 3: Python Virtual Environment**
- Creates venv at `~/venv-ardupilot`
- Uses `--system-site-packages` to access system Python packages
- Prompts to make venv default (adds to `.bashrc`)

**Step 4: Python Packages**
```
future lxml pymavlink MAVProxy pexpect argparse matplotlib pyparsing 
geocoder pyserial empy==3.3.4 dronecan packaging setuptools wheel
```

**Step 5: ARM Toolchain**
- Downloads GCC ARM None EABI 10-2020-q4-major
- Verifies SHA256 checksum: `21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618`
- Extracts to `/opt/gcc-arm-none-eabi-10-2020-q4-major/`
- Creates ccache symlinks in `/usr/lib/ccache/`

**Step 6: Environment Configuration**
```bash
# ARM toolchain
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH

# ArduPilot tools
export PATH=/path/to/ardupilot/Tools/autotest:$PATH

# Python venv activation
source $HOME/venv-ardupilot/bin/activate
```

**Step 7: Git Submodules**
- Initializes and updates all submodules

#### Post-Installation

**Important**: Log out and log back in for group membership (`uucp`) to take effect.

### Alpine Linux

**Script**: `install-prereqs-alpine.sh`  
**Source**: `/Tools/environment_install/install-prereqs-alpine.sh`

#### Overview

Alpine Linux support is **experimental** and provides minimal installation for containerized or embedded environments. This script installs only essential packages for SITL simulation.

**Warning**: This script is not fully tested. Please report issues to the ArduPilot GitHub repository.

#### Usage

```bash
cd ardupilot
Tools/environment_install/install-prereqs-alpine.sh
```

#### Packages Installed

**System Packages** (via apk):
```
linux-headers g++ python3 py-future py-pip libxml2-dev libxslt-dev git
```

**Python Packages** (via pip):
```
empy==3.3.4 pexpect ptyprocess
```

Note: Uses `--break-system-packages` flag to allow pip installations in system Python.

#### Limitations

- No ARM cross-compilation toolchain
- No graphical SITL support
- No MAVProxy (install manually if needed)
- Minimal Python package set
- Best suited for Docker containers or CI environments

### openSUSE Tumbleweed

**Script**: `install-prereqs-openSUSE-Tumbleweed.sh`  
**Source**: `/Tools/environment_install/install-prereqs-openSUSE-Tumbleweed.sh`

#### Features
- Rolling release support
- Zypper package management
- Both ARM None EABI and ARM Linux toolchains
- Python virtual environment with system-site-packages
- Enhanced ccache configuration

#### Usage

```bash
cd ardupilot
Tools/environment_install/install-prereqs-openSUSE-Tumbleweed.sh -y
```

**Command-Line Options**:
- `-y`: Assume yes to all prompts
- `-q`: Quiet mode for pip installations

#### Installation Process

**Step 1: Python Installation**
- Installs Python 3 (openSUSE TW WSL may not have Python preinstalled)
- Detects active Python version (e.g., `python311`, `python312`)

**Step 2: System Packages**
```bash
# Base development
patterns-devel-base-devel_basis ccache git axel valgrind screen gcc-c++ 
xterm free-ttf-fonts zip rsync glibc-devel-static

# SITL dependencies (version-specific Python packages)
python3XX-pip python3XX-devel python3XX-setuptools python3XX-wheel 
python3XX-lxml python3XX-pyaml python3XX-wxPython python3XX-pyparsing 
python3XX-opencv python3XX-numpy python3XX-scipy python3XX-matplotlib

# Graphics
sfml2-devel libgtk-3-dev libwxgtk3.2-dev
```

**Step 3: Python Virtual Environment**
- Creates venv at `~/venv-ardupilot`
- Activates automatically
- Prompts to add to `.bashrc`

**Step 4: Python Packages**
```
future lxml pymavlink MAVProxy pexpect argparse pyparsing geocoder 
pyserial empy==3.3.4 ptyprocess dronecan flake8 junitparser pygame 
intelhex psutil pyyaml attrdict3
```

**Step 5: Toolchain Installation**

**ARM None EABI** (STM32 boards):
- Downloads GCC ARM None EABI 10-2020-q4-major
- Installs to `/opt/gcc-arm-none-eabi-10-2020-q4-major/`

**ARM Linux** (ARM Linux targets):
- Downloads GCC Linaro 7.5.0-2019.12
- Installs to `/opt/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/`

**Step 6: CCache Configuration**
Creates symlinks in `/usr/lib64/ccache/` for:
```
arm-none-eabi-g++
arm-none-eabi-gcc
arm-linux-gnueabihf-g++
arm-linux-gnueabihf-gcc
```

Configures ccache for STM32 builds:
```bash
ccache --set-config sloppiness=file_macro,locale,time_macros
ccache --set-config ignore_options="--specs=nano.specs --specs=nosys.specs"
```

**Step 7: Package Removal**
Removes conflicting packages:
- `ModemManager`
- `brltty`

**Step 8: Environment Configuration**
Adds to `.profile`:
```bash
# ARM None EABI toolchain
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH

# ARM Linux toolchain
export PATH=/opt/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin:$PATH

# ArduPilot tools
export PATH=/path/to/ardupilot/Tools/autotest:$PATH

# CCache
export PATH=/usr/lib64/ccache:$PATH
```

Adds to `.bashrc`:
```bash
# Bash completion
source /path/to/ardupilot/Tools/completion/completion.bash

# Python venv
source $HOME/venv-ardupilot/bin/activate
```

**Step 9: Git Submodules**
- Updates all submodules if `.git` directory exists

#### Post-Installation

**Important**: Log out and log back in for group membership and environment changes to take effect.

## Components Installed

### Build System Components

| Component | Purpose | Platforms |
|-----------|---------|-----------|
| **GCC/G++** | C/C++ compiler for native builds | All Linux, macOS |
| **Make** | Build automation | All |
| **CMake** | Build system generator (ROS) | Ubuntu (with ROS) |
| **Waf** | ArduPilot build system (bundled) | All |
| **ccache** | Compiler cache for faster rebuilds | All |

### Python Environment

#### Core Python Packages

| Package | Version | Purpose |
|---------|---------|---------|
| **empy** | 3.3.4 | Template engine for code generation |
| **pymavlink** | Latest | MAVLink protocol library |
| **MAVProxy** | Latest | Command-line ground control station |
| **dronecan** | Latest | DroneCAN/UAVCAN protocol support |
| **pyserial** | Latest | Serial port communication |
| **future** | Latest | Python 2/3 compatibility |
| **lxml** | Latest | XML processing |
| **pexpect** | Latest | Process automation |
| **ptyprocess** | Latest | Pseudoterminal utilities |

#### SITL Simulation Packages

| Package | Purpose | Skip Variable |
|---------|---------|---------------|
| **pygame** | Graphical display simulation | SKIP_AP_EXT_ENV |
| **matplotlib** | Plotting and visualization | SKIP_AP_GRAPHIC_ENV |
| **scipy** | Scientific computing | SKIP_AP_GRAPHIC_ENV |
| **opencv-python** | Computer vision | SKIP_AP_GRAPHIC_ENV |
| **wxPython** | GUI toolkit for MAVProxy | SKIP_AP_GRAPHIC_ENV |
| **numpy** | Numerical computing | System package |
| **pyparsing** | Parser toolkit | System package |

#### Development and Testing Packages

| Package | Purpose |
|---------|---------|
| **flake8** | Python code linting |
| **junitparser** | Parse JUnit XML test reports |
| **geocoder** | Geocoding utilities |
| **intelhex** | Intel HEX file processing |
| **psutil** | System monitoring |
| **pyyaml** | YAML parsing |
| **wsproto** | WebSocket protocol |
| **tabulate** | Table formatting |

#### Code Coverage Tools (Optional)

| Tool | Purpose | Platform |
|------|---------|----------|
| **lcov** | Line coverage visualization | Ubuntu/Debian |
| **gcovr** | Coverage report generator | Ubuntu/Debian |

### ARM Toolchains

#### STM32 Boards (ARM None EABI)

**Toolchain**: GCC ARM None EABI 10-2020-q4-major  
**Install Location**: `/opt/gcc-arm-none-eabi-10-2020-q4-major/`  
**Source**: https://firmware.ardupilot.org/Tools/STM32-tools/

**Targets**:
- STM32F1, STM32F3, STM32F4, STM32F7, STM32H7 microcontrollers
- All ChibiOS-based autopilot boards
- Examples: Pixhawk, Cube, Kakute, Matek, Holybro boards

**Architecture Support**:
- x86_64 Linux
- aarch64 Linux (ARM64 host)
- macOS (Intel and Apple Silicon via Rosetta 2)
- Windows (32-bit)

**Compiler Specifications**:
- GCC Version: 10.2.1
- Binutils: 2.35.1
- Newlib: 3.3.0
- GDB: 10.1

#### ARM Linux Targets (ARM Linux GNUEABIHF)

**Toolchain**: GCC Linaro 7.5.0-2019.12 (openSUSE) or distribution package (Ubuntu/Debian)  
**Install Location**: `/opt/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/` (openSUSE)

**Targets**:
- BeagleBone Blue/Black
- Raspberry Pi (with ArduPilot on Linux)
- NVIDIA Jetson boards
- Other ARM Linux-based systems

**Available on**:
- Ubuntu/Debian (via `g++-arm-linux-gnueabihf` package)
- openSUSE (downloaded toolchain)

### Ground Control Software

#### MAVProxy

**Platform**: All  
**Installation Method**: 
- Linux: pip package
- macOS: pip package
- Windows: Native installer

**Features**:
- Command-line ground control station
- MAVLink message inspection
- Module system for extensibility
- SITL simulation integration
- Real-time telemetry display

**Modules Installed** (with graphical support):
- Map display (requires wxPython)
- Console (terminal interface)
- Graph plotting (matplotlib)
- Video processing (opencv)

## Python Virtual Environments

### When Virtual Environments Are Created

Python virtual environments (venv) are automatically created on:

| Platform | Condition | Location |
|----------|-----------|----------|
| **Ubuntu** | 23.04 Lunar and newer | `~/venv-ardupilot` |
| **Debian** | Bookworm (12) and newer | `~/venv-ardupilot` |
| **Arch Linux** | Always | `~/venv-ardupilot` |
| **openSUSE** | Always | `~/venv-ardupilot` |
| **macOS** | If using pyenv | Managed by pyenv |
| **Windows** | Never (Cygwin Python) | N/A |

### Rationale

Recent Linux distributions (PEP 668) restrict pip installations in system Python to prevent conflicts with distribution packages. Virtual environments provide isolated Python package installations.

### Configuration

**System-Site-Packages Access**:
All venvs are created with `--system-site-packages` flag, allowing access to distribution-provided packages (numpy, scipy, etc.) while installing additional packages in the venv.

**Activation Methods**:

**Manual Activation**:
```bash
source ~/venv-ardupilot/bin/activate
```

**Automatic Activation** (if selected during installation):
Added to shell configuration (`.bashrc`, `.profile`, `.zshrc`):
```bash
source $HOME/venv-ardupilot/bin/activate
```

### Working with Virtual Environments

**Check if venv is active**:
```bash
echo $VIRTUAL_ENV
# Output: /home/username/venv-ardupilot
```

**Install additional packages**:
```bash
# Venv must be activated
pip install package_name
```

**Deactivate venv**:
```bash
deactivate
```

**Recreate venv** (if corrupted):
```bash
rm -rf ~/venv-ardupilot
python3 -m venv --system-site-packages ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install -U pip
# Reinstall ArduPilot packages
pip install pymavlink MAVProxy dronecan empy==3.3.4 ...
```

## ARM Toolchains

### STM32 Toolchain Details

**Installation Path**: `/opt/gcc-arm-none-eabi-10-2020-q4-major/`

**Binaries Provided**:
```
arm-none-eabi-gcc        # C compiler
arm-none-eabi-g++        # C++ compiler
arm-none-eabi-objcopy    # Binary conversion tool
arm-none-eabi-size       # Size information utility
arm-none-eabi-gdb        # GNU debugger
arm-none-eabi-as         # Assembler
arm-none-eabi-ld         # Linker
```

**CCache Integration**:
The installation scripts create symlinks for compiler caching:
```bash
# Ubuntu/Debian
/usr/lib/ccache/arm-none-eabi-gcc -> /usr/bin/ccache
/usr/lib/ccache/arm-none-eabi-g++ -> /usr/bin/ccache

# macOS
/usr/local/opt/ccache/libexec/arm-none-eabi-gcc -> /path/to/ccache
/usr/local/opt/ccache/libexec/arm-none-eabi-g++ -> /path/to/ccache

# Arch/openSUSE
/usr/lib/ccache/arm-none-eabi-gcc -> /usr/bin/ccache
/usr/lib/ccache/arm-none-eabi-g++ -> /usr/bin/ccache
```

**Building for STM32 Boards**:
```bash
./waf configure --board=MatekH743
./waf build --target bin/arducopter
```

**Supported Board Families**:
- Pixhawk 1/2/4/5/6 series
- Cube Orange/Yellow/Purple/Black
- Holybro Pixhawk/Durandal/Pix32 series
- Matek H7 series (H743, H755, H757)
- Kakute F4/F7/H7 series
- CUAV V5/V5+/Nano/X7
- mRo Pixhawk variants
- Hex (ProfiCNC) boards
- JHEMCU GSF4/GSF7 series
- And 100+ other ChibiOS boards

### ARM Linux Toolchain Details

**Purpose**: Cross-compile ArduPilot to run on ARM Linux boards (not bare-metal microcontrollers).

**Installation**:
- Ubuntu/Debian: `g++-arm-linux-gnueabihf` package
- openSUSE: GCC Linaro 7.5.0-2019.12 in `/opt/`

**Building for ARM Linux**:
```bash
./waf configure --board=linux
./waf build --target bin/arducopter
```

**Target Devices**:
- BeagleBone Blue (recommended Linux board)
- Raspberry Pi 2/3/4 with PXF Cape
- NVIDIA Jetson Nano/TX2/Xavier
- Generic ARM Linux systems

**Note**: ARM Linux builds run ArduPilot as a user-space application on Linux, not on bare-metal like STM32 builds.

## ROS Integration

### ROS Installation Script

**Script**: `install-ROS-ubuntu.sh`  
**Source**: `/Tools/environment_install/install-ROS-ubuntu.sh`

#### Overview

Installs ROS 1 (Robot Operating System) and configures integration with ArduPilot for robotics applications. This is an optional component for users integrating ArduPilot with ROS-based systems.

**Supported ROS Distributions**:
- **ROS Noetic** on Ubuntu 20.04 Focal

**Unsupported**:
- Ubuntu 22.04+ only supports ROS 2 (not covered by this script)

#### Usage

```bash
cd ardupilot/Tools/environment_install
./install-ROS-ubuntu.sh
```

**With Custom Packages**:
```bash
./install-ROS-ubuntu.sh -p desktop-full
```

#### Installation Steps

**Step 1: ROS Base Installation**
- Adds ROS apt repositories
- Installs ROS Noetic desktop-full (or specified package)
- Configures apt keys and sources

**Step 2: MAVROS Installation**
- Installs `ros-noetic-mavros`
- Downloads GeographicLib datasets (required for GPS coordinate transformations)

**Step 3: rosdep Initialization**
- Installs rosdep for dependency management
- Initializes rosdep database
- Updates rosdep package lists

**Step 4: ArduPilot ROS Workspace** (optional, if accepted)
- Creates `~/ardupilot-ws` catkin workspace
- Clones `ardupilot_ros` repository
- Installs workspace dependencies via rosdep
- Builds workspace with catkin

**Step 5: ArduPilot Gazebo Integration** (optional, if accepted)
- Installs Gazebo Garden simulation environment
- Creates `~/ardupilot_gz_ws` workspace
- Clones and builds `ardupilot_gazebo` plugin
- Configures Gazebo environment variables

**Step 6: Environment Configuration**
Adds to `.bashrc`:
```bash
# ROS environment
ROS_HOSTNAME=hostname.local
ROS_MASTER_URI=http://hostname.local:11311
source /opt/ros/noetic/setup.bash

# ArduPilot ROS workspace
source ~/ardupilot-ws/devel/setup.bash

# Gazebo plugin paths
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=~/ardupilot_gz_ws/src/ardupilot_gazebo/models:~/ardupilot_gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
```

#### Using ROS with ArduPilot

**Launch MAVROS with SITL**:
```bash
# Terminal 1: Start SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map

# Terminal 2: Launch MAVROS
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@
```

**Test ROS Topics**:
```bash
rostopic list
rostopic echo /mavros/state
rosservice call /mavros/cmd/arming "value: true"
```

#### Known Limitations

- ROS 1 only supported on Ubuntu 20.04 Focal
- ROS 2 support requires separate setup (not automated)
- Gazebo Garden integration is experimental

### APM Source Installation

**Script**: `APM_install.sh`  
**Source**: `/Tools/environment_install/APM_install.sh`

#### Purpose

Simple script for cloning ArduPilot source and performing initial configuration. Primarily used by Windows installation scripts within Cygwin environment.

#### What It Does

```bash
# Clone ArduPilot repository
git clone git://github.com/ArduPilot/ardupilot.git

# Navigate to repository
cd ./ardupilot

# Initialize and update all submodules
git submodule update --init --recursive

# Configure waf build system for SITL
./modules/waf/waf-light configure --board=sitl
```

#### Usage

**Standalone** (Linux/macOS):
```bash
cd ~
Tools/environment_install/APM_install.sh
```

**Automatic** (Windows):
- Called automatically by `install-prereqs-windows-andAPMSource.ps1`
- Executed within Cygwin bash environment

## Environment Variables

### PATH Modifications

The installation scripts modify PATH to include:

| Path Component | Purpose | Added By |
|----------------|---------|----------|
| `/opt/gcc-arm-none-eabi-*/bin` | ARM compiler binaries | All platforms (if accepted) |
| `~/ardupilot/Tools/autotest` | SITL simulation tools | All platforms |
| `/usr/lib/ccache` (Linux) | Compiler cache binaries | Linux platforms |
| `/usr/local/opt/ccache/libexec` (macOS) | Compiler cache binaries | macOS |

### Shell Configuration Files Modified

| Platform | File Modified | Purpose |
|----------|---------------|---------|
| **Ubuntu/Debian** | `~/.profile` | Login shell environment |
| **Ubuntu/Debian** | `~/.bashrc` | Interactive bash config |
| **Ubuntu (Docker)** | `~/.ardupilot_env` | Container-specific env |
| **macOS (bash)** | `~/.bash_profile` | Bash login environment |
| **macOS (zsh)** | `~/.zshrc` | Zsh interactive config |
| **Arch/openSUSE** | `~/.profile` | Login shell environment |
| **Arch/openSUSE** | `~/.bashrc` | Interactive bash config |

### ROS-Specific Variables

If ROS is installed:

```bash
ROS_HOSTNAME=hostname.local          # ROS node hostname
ROS_MASTER_URI=http://hostname.local:11311  # ROS master URI
GZ_SIM_SYSTEM_PLUGIN_PATH=...        # Gazebo plugin path
GZ_SIM_RESOURCE_PATH=...             # Gazebo resource path
```

### Python Virtual Environment

If venv is configured:

```bash
VIRTUAL_ENV=/home/user/venv-ardupilot  # Active venv path
PATH=$VIRTUAL_ENV/bin:$PATH            # Venv binaries first
```

### Pyenv (macOS)

If pyenv is installed:

```bash
PYENV_ROOT=$HOME/.pyenv              # Pyenv installation root
PATH=$PYENV_ROOT/bin:$PATH           # Pyenv binary path
# Shell initialization hooks
eval "$(pyenv init --path)"
eval "$(pyenv init -)"
```

## Troubleshooting

### Common Issues and Solutions

#### Permission Denied on Serial Ports

**Symptom**:
```
Error: Could not open port /dev/ttyACM0: Permission denied
```

**Solution**:
User not in correct group for serial port access.

**Linux (Ubuntu/Debian/openSUSE)**:
```bash
# Check current groups
groups

# Should see 'dialout' in the list
# If not, run the install script again or:
sudo usermod -a -G dialout $USER

# Log out and log back in (group membership)
```

**Linux (Arch)**:
```bash
# Arch uses 'uucp' group instead of 'dialout'
sudo usermod -a -G uucp $USER
# Log out and log back in
```

**Verify**:
```bash
# Check port permissions
ls -l /dev/ttyACM0
# Should show: crw-rw---- 1 root dialout ...
```

#### Python Package Installation Fails

**Symptom**:
```
error: externally-managed-environment
```

**Cause**: PEP 668 restriction on system Python (Ubuntu 23.04+, Debian Bookworm+)

**Solution**:
Ensure virtual environment is activated:
```bash
source ~/venv-ardupilot/bin/activate
pip install package_name
```

Or use `--break-system-packages` flag (not recommended):
```bash
pip install --break-system-packages package_name
```

#### wxPython Installation Takes Very Long Time

**Symptom**: Installation appears frozen during wxPython install

**Cause**: wxPython compiles from source on some platforms (can take 20-40 minutes)

**Solution**: Be patient. The script warns about this:
```
##### wxpython takes a *VERY* long time to install (~30 minutes). Be patient.
```

Monitor with:
```bash
# In another terminal
top  # Check if gcc/g++ are running
```

#### ARM Toolchain Not Found

**Symptom**:
```bash
./waf configure --board=MatekH743
# Error: arm-none-eabi-gcc not found
```

**Solution**:

**Check installation**:
```bash
ls /opt/gcc-arm-none-eabi-10-2020-q4-major/
```

**Check PATH**:
```bash
echo $PATH | grep arm-none-eabi
```

**Add to PATH manually**:
```bash
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
./waf configure --board=MatekH743
```

**Permanent fix**:
Re-run installation script and accept PATH modification when prompted.

#### Git Submodule Errors

**Symptom**:
```
fatal: not a git repository
```

**Cause**: Running script outside ArduPilot repository

**Solution**:
```bash
cd /path/to/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh
```

**Or skip submodule update**:
```bash
export SKIP_AP_GIT_CHECK=1
Tools/environment_install/install-prereqs-ubuntu.sh
```

#### CCache Not Working

**Symptom**: Builds don't seem to use cache (always slow)

**Check ccache is installed**:
```bash
which ccache
ccache --version
```

**Check ccache statistics**:
```bash
ccache -s
# Should show hits/misses
```

**Check PATH order**:
```bash
echo $PATH
# ccache path should appear before /usr/bin
```

**Verify symlinks** (Ubuntu):
```bash
ls -la /usr/lib/ccache/
# Should show symlinks to ccache for gcc, g++, arm-none-eabi-gcc, etc.
```

**Reset ccache**:
```bash
ccache -C  # Clear cache
ccache -z  # Zero statistics
```

#### ModemManager Interfering with Uploads

**Symptom**: Firmware upload fails or disconnects autopilot

**Cause**: ModemManager probes serial devices

**Solution**:
```bash
# Check if installed
dpkg -l | grep modemmanager

# Remove (installation script should do this)
sudo apt remove modemmanager

# Or disable temporarily
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
```

#### macOS Homebrew Issues

**Symptom**: `brew update` or `brew install` fails

**Solution**:

**Permission issues**:
```bash
# Fix Homebrew permissions
sudo chown -R $(whoami) /usr/local/bin /usr/local/lib /usr/local/share
```

**Xcode issues**:
```bash
# Reinstall Command Line Tools
sudo rm -rf /Library/Developer/CommandLineTools
xcode-select --install
```

**Python framework conflicts**:
```bash
# Remove conflicting Python links (script does this)
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete
```

#### Windows Cygwin Path Issues

**Symptom**: Cannot find files or commands in Cygwin

**Cause**: Windows paths (`C:\`) not directly accessible in Cygwin

**Solution**: Use Cygwin paths:
- Windows `C:\Users\username` = Cygwin `/cygdrive/c/Users/username`
- Cygwin home: `/home/username` or `~`

**Navigate to Windows directories**:
```bash
cd /cygdrive/c/Users/YourName/Documents
```

#### Insufficient Disk Space

**Symptom**: Installation fails partway through

**Check space**:
```bash
df -h
```

**Required space**:
- Minimal: ~2GB (Alpine)
- Standard: ~5GB (Ubuntu without toolchains)
- Full: ~8GB (Ubuntu with all toolchains and ROS)

**Free up space**:
```bash
# Clean apt cache (Ubuntu/Debian)
sudo apt clean

# Clean ccache
ccache -C

# Remove old kernels (Ubuntu)
sudo apt autoremove
```

## Advanced Configuration

### Customizing Installation Components

#### Selective Component Installation

Use environment variables to control what gets installed:

```bash
# Example: Minimal headless SITL installation (no GUI, no extras)
export SKIP_AP_GRAPHIC_ENV=1      # Skip matplotlib, opencv, wxPython
export SKIP_AP_EXT_ENV=1          # Skip pygame, intelhex
export SKIP_AP_COV_ENV=1          # Skip coverage tools
export DO_AP_STM_ENV=0            # Don't install STM32 toolchain
export SKIP_AP_COMPLETION_ENV=1   # Skip bash completion
export DO_PYTHON_VENV_ENV=1       # Force venv creation

Tools/environment_install/install-prereqs-ubuntu.sh -y
```

```bash
# Example: Full installation with everything
unset SKIP_AP_GRAPHIC_ENV
unset SKIP_AP_EXT_ENV
unset SKIP_AP_COV_ENV
export DO_AP_STM_ENV=1            # Install STM32 toolchain
export DO_PYTHON_VENV_ENV=1       # Use Python venv

Tools/environment_install/install-prereqs-ubuntu.sh -y
```

#### Docker/Container Optimization

For containerized environments:

```bash
# Set Docker environment
export AP_DOCKER_BUILD=1

# Skip git operations (handled externally)
export SKIP_AP_GIT_CHECK=1

# Use minimal packages
export SKIP_AP_GRAPHIC_ENV=1

# Run installation
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

The script creates `~/.ardupilot_env` instead of modifying `.profile` in Docker mode.

**Dockerfile Example**:
```dockerfile
FROM ubuntu:22.04

ENV AP_DOCKER_BUILD=1
ENV SKIP_AP_GRAPHIC_ENV=1
ENV DO_AP_STM_ENV=1

COPY ardupilot /ardupilot
WORKDIR /ardupilot

RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Activate environment for all shells
RUN echo "source ~/.ardupilot_env" >> /root/.bashrc

CMD ["/bin/bash"]
```

### CCache Configuration

#### Increase Cache Size

Default ccache size may be too small for multiple board builds:

```bash
# Check current size and usage
ccache -s

# Set cache size to 10GB
ccache -M 10G

# Set maximum file size
ccache -F 10000  # 10000 files
```

#### Configure ccache for Optimal Performance

```bash
# Set compression level (0-9, default 6)
ccache -o compression_level=6

# Set sloppiness for faster hits (STM32 builds)
ccache -o sloppiness=file_macro,locale,time_macros

# Ignore specific compiler flags (STM32 specs)
ccache -o ignore_options="--specs=nano.specs --specs=nosys.specs"
```

#### Per-Project ccache Directories

```bash
# Use separate cache for ArduPilot
export CCACHE_DIR=/path/to/ardupilot/.ccache
ccache -M 5G
```

### Multiple Python Environments

#### Using Multiple Virtual Environments

Create separate venvs for different purposes:

```bash
# ArduPilot development venv
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install pymavlink MAVProxy dronecan empy==3.3.4

# ROS development venv
python3 -m venv ~/venv-ros
source ~/venv-ros/bin/activate
pip install rosdep catkin-tools

# Testing/CI venv
python3 -m venv ~/venv-ci
source ~/venv-ci/bin/activate
pip install pytest coverage flake8
```

**Switch between venvs**:
```bash
deactivate              # Deactivate current venv
source ~/venv-ros/bin/activate  # Activate different venv
```

#### macOS pyenv Multiple Versions

```bash
# Install multiple Python versions
pyenv install 3.9.16
pyenv install 3.10.11
pyenv install 3.11.3

# Set global version
pyenv global 3.10.11

# Set local version for specific directory
cd ~/ardupilot
pyenv local 3.10.11

# Use specific version temporarily
pyenv shell 3.9.16
python --version
```

### Cross-Compilation Optimization

#### Building for Multiple Boards Efficiently

**Use ccache** (already configured by install scripts)

**Parallel builds**:
```bash
# Build with all CPU cores
./waf build -j$(nproc)

# Build with specific number of jobs
./waf build -j8
```

**Build multiple boards**:
```bash
# Build script for CI
for BOARD in Pixhawk1 MatekH743 KakuteH7 CubeOrange; do
    ./waf configure --board=$BOARD
    ./waf build --target bin/arducopter -j$(nproc)
done
```

#### Reducing Build Time

**Clean only when necessary**:
```bash
# Clean entire build
./waf clean

# Clean only specific board
./waf configure --board=MatekH743
./waf clean

# Partial rebuild (faster)
./waf build --target bin/arducopter  # Rebuilds only changed files
```

**Use build directories**:
```bash
# Keep separate build directories for different boards
./waf configure --board=Pixhawk1 --out=build-pixhawk1
./waf configure --board=MatekH743 --out=build-matekh743
# Switching boards doesn't require full rebuild
```

### Network Configuration

#### Offline Installation Preparation

Download packages for offline installation:

**Ubuntu/Debian**:
```bash
# Download packages without installing
mkdir ~/ardupilot-offline-packages
cd ~/ardupilot-offline-packages

apt-get download build-essential ccache git python3-pip python3-dev

# Download Python packages
pip download pymavlink MAVProxy dronecan empy==3.3.4

# Download ARM toolchain
wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
```

**Install offline**:
```bash
# Install system packages
sudo dpkg -i *.deb
sudo apt-get install -f  # Fix dependencies

# Install Python packages
pip install --no-index --find-links=. pymavlink MAVProxy dronecan empy==3.3.4

# Extract toolchain
sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -C /opt/
```

#### Using Custom Mirrors

**Ubuntu/Debian**:
Edit script to use custom mirror:
```bash
# Edit install-prereqs-ubuntu.sh
# Change APT_GET or add mirror configuration
APT_GET="sudo apt-get -o Acquire::http::Proxy=\"http://proxy:3142\""
```

**Python packages**:
```bash
# Use custom PyPI mirror
export PIP_INDEX_URL=http://pypi.internal/simple/
pip install pymavlink
```

## Maintenance and Updates

### Updating Python Packages

#### Upgrading All Packages

```bash
# Activate venv (if using)
source ~/venv-ardupilot/bin/activate

# Update pip itself
pip install --upgrade pip

# Update all ArduPilot-related packages
pip install --upgrade pymavlink MAVProxy dronecan empy pexpect matplotlib scipy opencv-python
```

#### Checking for Outdated Packages

```bash
# List outdated packages
pip list --outdated

# Update specific package
pip install --upgrade pymavlink
```

#### MAVProxy Updates

MAVProxy is frequently updated with new features:

```bash
# Check current version
mavproxy.py --version

# Update to latest
pip install --upgrade MAVProxy

# Install specific version
pip install MAVProxy==1.8.55
```

### Updating ARM Toolchains

#### Check Current Version

```bash
arm-none-eabi-gcc --version
# gcc version 10.2.1 20201103 (release)
```

#### Manual Toolchain Update

If a newer toolchain is released:

```bash
# Download new toolchain
cd /tmp
wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-XX-XXXX-qX-major-x86_64-linux.tar.bz2

# Extract to /opt
sudo tar xjf gcc-arm-none-eabi-XX-XXXX-qX-major-x86_64-linux.tar.bz2 -C /opt/

# Update PATH in shell config
# Edit ~/.profile or ~/.bashrc
export PATH=/opt/gcc-arm-none-eabi-XX-XXXX-qX-major/bin:$PATH

# Update ccache symlinks
sudo ln -sf $(which ccache) /usr/lib/ccache/arm-none-eabi-gcc
sudo ln -sf $(which ccache) /usr/lib/ccache/arm-none-eabi-g++

# Verify
arm-none-eabi-gcc --version
```

### Updating System Packages

#### Ubuntu/Debian

```bash
# Update package lists
sudo apt update

# Upgrade installed packages
sudo apt upgrade

# Upgrade distribution (major version)
sudo apt dist-upgrade
```

**After system upgrade**, re-run ArduPilot installation script to ensure compatibility:
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

#### Arch Linux

```bash
# Update all packages (rolling release)
sudo pacman -Syu

# Update Python packages in venv
source ~/venv-ardupilot/bin/activate
pip install --upgrade pymavlink MAVProxy dronecan
```

#### macOS

```bash
# Update Homebrew
brew update

# Upgrade installed packages
brew upgrade

# Update Python packages
pip install --upgrade pymavlink MAVProxy dronecan
```

### Re-running Installation Scripts

Installation scripts are idempotent and can be safely re-run:

```bash
# Re-run to update components
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

**What gets updated**:
- System packages (via apt/brew/pacman)
- Python packages (latest versions)
- Git submodules

**What doesn't change**:
- ARM toolchain (if already installed)
- Shell configuration (if already configured)
- Virtual environment (if already exists)

### Cleaning Up Old Installations

#### Remove Old Toolchains

```bash
# List installed toolchains
ls -la /opt/ | grep gcc-arm

# Remove old version
sudo rm -rf /opt/gcc-arm-none-eabi-OLD-VERSION

# Clean up PATH references in shell config
# Edit ~/.profile, ~/.bashrc, etc.
```

#### Clean Build Artifacts

```bash
cd ardupilot

# Clean all builds
./waf distclean

# Remove ccache
ccache -C

# Remove Python cache
find . -type d -name __pycache__ -exec rm -rf {} +
find . -type f -name "*.pyc" -delete
```

#### Recreate Environment from Scratch

```bash
# Remove virtual environment
rm -rf ~/venv-ardupilot

# Remove shell configuration (backup first!)
cp ~/.profile ~/.profile.backup
cp ~/.bashrc ~/.bashrc.backup

# Re-run installation
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Validation After Updates

#### Verify Installation

```bash
# Check Python packages
python3 -c "import pymavlink; print(pymavlink.__version__)"
python3 -c "import MAVProxy; print('MAVProxy OK')"

# Check ARM toolchain
arm-none-eabi-gcc --version

# Check ccache
ccache -s

# Check git submodules
cd ardupilot
git submodule status
```

#### Test SITL Build

```bash
cd ardupilot/ArduCopter
./waf configure --board=sitl
./waf build --target bin/arducopter

# Run SITL
sim_vehicle.py --console --map
```

#### Test ARM Build

```bash
cd ardupilot
./waf configure --board=Pixhawk1
./waf build --target bin/arducopter

# Verify binary was created
ls -lh build/Pixhawk1/bin/arducopter.apj
```

## Additional Resources

### Documentation

- **ArduPilot Wiki**: https://ardupilot.org/dev/
- **Building the Code**: https://ardupilot.org/dev/docs/building-the-code.html
- **SITL Simulator**: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- **MAVProxy**: https://ardupilot.org/mavproxy/

### Community Support

- **ArduPilot Discourse**: https://discuss.ardupilot.org/
- **Development Category**: https://discuss.ardupilot.org/c/development/
- **Discord**: https://ardupilot.org/discord

### Issue Reporting

If you encounter issues with environment setup scripts:

1. **Search existing issues**: https://github.com/ArduPilot/ardupilot/issues
2. **Report new issue**: Include:
   - Operating system and version (`lsb_release -a` on Linux)
   - Script name and version
   - Full error output
   - Installation log

### Contributing

Improvements to installation scripts are welcome:

1. Test on your platform
2. Submit pull request: https://github.com/ArduPilot/ardupilot/pulls
3. Update documentation in this README
4. Follow ArduPilot contribution guidelines

---

**Last Updated**: 2025  
**Script Versions**: Compatible with ArduPilot master branch  
**Maintainers**: ArduPilot Development Team

For the latest version of this documentation, see:  
https://github.com/ArduPilot/ardupilot/tree/master/Tools/environment_install
