# ArduPilot Project

<a href="https://ardupilot.org/discord"><img src="https://img.shields.io/discord/674039678562861068.svg" alt="Discord">

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml) [![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_plane.yml) [![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_rover.yml) [![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_sub.yml) [![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_tracker.yml)

[![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_periph.yml) [![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_chibios.yml) [![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_linux_sbc.yml) [![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_replay.yml)

[![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests%20and%20sitl%20building/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_unit_tests.yml)[![test size](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml)

[![Test Environment Setup](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml)

[![Cygwin Build](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml) [![Macos Build](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Test Coverage](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

ArduPilot is the most advanced, full-featured, and reliable open source autopilot software available.
It has been under development since 2010 by a diverse team of professional engineers, computer scientists, and community contributors.
Our autopilot software is capable of controlling almost any vehicle system imaginable, from conventional airplanes, quad planes, multi-rotors, and helicopters to rovers, boats, balance bots, and even submarines.
It is continually being expanded to provide support for new emerging vehicle types.

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Safety-Critical Systems](#safety-critical-systems)
- [Getting Started](#getting-started)
- [Documentation Structure](#documentation-structure)
- [Vehicle Types](#vehicle-types)
- [Build System](#build-system)
- [Testing and Simulation](#testing-and-simulation)
- [Hardware Compatibility](#hardware-compatibility)
- [User Support & Discussion Forums](#user-support--discussion-forums)
- [Developer Information](#developer-information)
- [Contributing](#contributing)
- [Top Contributors](#top-contributors)
- [License](#license)
- [Maintainers](#maintainers)

## Overview

ArduPilot represents a comprehensive autopilot ecosystem that provides flight control, navigation, and autonomous operation capabilities for unmanned vehicles across multiple domains. The system implements advanced control algorithms, sensor fusion, mission planning, and failsafe mechanisms to ensure safe and reliable autonomous operation.

### Key Capabilities

- **Multi-Vehicle Support**: Unified architecture supporting copters, planes, rovers, submarines, antenna trackers, and experimental vehicles
- **Advanced Navigation**: Extended Kalman Filter (EKF) based state estimation with multi-sensor fusion
- **Flexible Mission System**: Waypoint navigation, geo-fencing, rally points, and complex mission commands
- **Robust Communication**: MAVLink protocol, DroneCAN/UAVCAN, telemetry streaming, and ground station integration
- **Extensive Hardware Support**: 100+ autopilot boards with Hardware Abstraction Layer (HAL) for portability
- **Safety-Critical Design**: Multi-level failsafe systems, pre-arm checks, and health monitoring

## System Architecture

ArduPilot is built on a modular architecture with clear separation between vehicle-specific logic and shared libraries:

### Core Components

**Vehicle Implementations** - Vehicle-specific control systems located in:
- `ArduCopter/` - Multirotor and helicopter flight control
- `ArduPlane/` - Fixed-wing and VTOL aircraft
- `Rover/` - Ground and surface vehicles
- `ArduSub/` - Underwater ROV/AUV systems
- `AntennaTracker/` - Antenna tracking systems
- `Blimp/` - Lighter-than-air vehicles

**Shared Libraries** (`libraries/`) - Reusable components including:
- **Hardware Abstraction**: `AP_HAL`, `AP_HAL_ChibiOS`, `AP_HAL_Linux`, `AP_HAL_SITL`
- **Sensor Management**: `AP_InertialSensor`, `AP_Compass`, `AP_Baro`, `AP_GPS`, `AP_RangeFinder`
- **Navigation & Control**: `AP_AHRS`, `AP_NavEKF2`, `AP_NavEKF3`, `AC_AttitudeControl`, `AC_WPNav`
- **Communication**: `GCS_MAVLink`, `AP_DroneCAN`, `AP_DDS`
- **Actuation**: `AP_Motors`, `SRV_Channel`, `AP_BLHeli`
- **Mission Management**: `AP_Mission`, `AP_Rally`, `AC_Fence`

**Build System** - Waf-based build system in `Tools/ardupilotwaf/` supporting cross-compilation for multiple platforms

**Development Tools** - Comprehensive tooling in `Tools/` including:
- Autotest framework for automated testing
- Replay tools for log analysis
- Environment setup scripts
- SITL (Software In The Loop) simulation

### Data Flow Architecture

```
Sensors → HAL → Sensor Drivers → AHRS/EKF → Control Loops → Motor Mixing → Hardware Outputs
            ↓                                      ↑
        Scheduler                          Mission/Mode Logic
            ↓                                      ↑
      GCS MAVLink ←→ Parameters ←→ Logging → DataFlash
```

## Safety-Critical Systems

ArduPilot implements multiple layers of safety systems to ensure reliable operation:

### Pre-Flight Safety
- **Arming Checks** (`libraries/AP_Arming/`) - Comprehensive pre-arm validation including sensor health, calibration status, GPS lock, and system readiness
- **Parameter Validation** - Range checking and interdependency validation for all configuration parameters

### In-Flight Safety
- **Failsafe Systems** - Multi-level failsafe triggers for radio loss, battery critical, GPS failure, EKF failure, and geofence violations
- **Health Monitoring** - Continuous monitoring of sensor health, EKF variance, vibration levels, and system performance
- **Flight Mode Restrictions** - Mode-specific safety constraints and transition validation

### Emergency Response
- **Failsafe Actions** - Configurable responses including Return-To-Launch (RTL), Land, Hold position, or controlled termination
- **Geofencing** (`AC_Fence`) - Inclusion and exclusion zones with automatic enforcement
- **Rally Points** (`AP_Rally`) - Safe landing location alternatives

For detailed safety documentation, see [SAFETY_CRITICAL.md](SAFETY_CRITICAL.md).

## Getting Started

### For End Users
1. **Choose Your Vehicle Type**: Review vehicle-specific documentation below
2. **Select Hardware**: Check [Hardware Compatibility](#hardware-compatibility) section
3. **Follow Setup Guide**: Visit the vehicle-specific wiki for detailed setup instructions
4. **Join Community**: Connect via [Discord](https://ardupilot.org/discord) and [forums](https://discuss.ardupilot.org/)

### For Developers
1. **Setup Development Environment**: See `Tools/environment_install/` for automated setup scripts
2. **Build Your First Target**: Follow [BUILD_SYSTEM.md](BUILD_SYSTEM.md) for build instructions
3. **Explore SITL**: Start with Software-In-The-Loop simulation for safe testing
4. **Review Developer Documentation**: Visit [ardupilot.org/dev/](https://ardupilot.org/dev/)
5. **Contribute**: See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines

### For Hardware Integrators
1. **Review HAL Architecture**: See `libraries/AP_HAL/README.md`
2. **Study Porting Guide**: Review `libraries/AP_HAL/PORTING_GUIDE.md`
3. **Examine Existing Board Definitions**: Check `libraries/AP_HAL_ChibiOS/hwdef/`
4. **Contact Development Team**: Join developer discussions on Discord

## Documentation Structure

ArduPilot documentation is organized hierarchically to serve different audiences:

### Repository Documentation
- **Root README.md** (this file) - Project overview and navigation
- **BUILD_SYSTEM.md** - Comprehensive build system documentation
- **CONTRIBUTING.md** - Contribution guidelines and coding standards
- **SAFETY_CRITICAL.md** - Safety-critical system documentation

### Vehicle-Specific Documentation
Each vehicle directory contains comprehensive documentation:
- `ArduCopter/README.md` - Copter architecture and flight modes
- `ArduCopter/FLIGHT_MODES.md` - Detailed flight mode documentation
- `ArduPlane/README.md` - Plane architecture and control systems
- `ArduPlane/QUADPLANE.md` - VTOL/QuadPlane technical specifications
- `Rover/README.md` - Ground vehicle systems
- `ArduSub/README.md` - Underwater vehicle systems

### Library Documentation
Each major library includes detailed documentation:
- `libraries/AP_HAL/README.md` - Hardware Abstraction Layer
- `libraries/AP_HAL/PORTING_GUIDE.md` - Board porting guide
- `libraries/AP_InertialSensor/README.md` - IMU management
- `libraries/AP_GPS/README.md` - GPS driver architecture
- `libraries/AP_Motors/README.md` - Motor control and mixing
- `libraries/GCS_MAVLink/README.md` - MAVLink implementation
- `libraries/AP_NavEKF3/README.md` - Extended Kalman Filter
- See individual library directories for complete documentation

### Technical Architecture Documentation
System-wide architecture documentation in `docs/`:
- `docs/ARCHITECTURE.md` - Overall system architecture
- `docs/COORDINATE_SYSTEMS.md` - Frame conventions and transformations
- `docs/GLOSSARY.md` - Terms and acronyms
- `docs/api/` - Doxygen-generated API documentation

### External Documentation
- **User Wikis**: [ardupilot.org](https://ardupilot.org) - Vehicle-specific user guides
- **Developer Wiki**: [ardupilot.org/dev/](https://ardupilot.org/dev/) - Development documentation
- **API Reference**: Auto-generated Doxygen documentation

## Vehicle Types

### Supported Vehicle Types

ArduPilot provides complete autopilot solutions for the following vehicle types:

#### ArduCopter - Multirotor and Helicopter
- **Code**: [ArduCopter/](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter)
- **Documentation**: [ArduCopter/README.md](ArduCopter/README.md), [FLIGHT_MODES.md](ArduCopter/FLIGHT_MODES.md)
- **User Wiki**: [ardupilot.org/copter/](https://ardupilot.org/copter/index.html)
- **Features**: 26+ flight modes, traditional helicopters, multirotors, autotuning, advanced position control

#### ArduPlane - Fixed-Wing and VTOL
- **Code**: [ArduPlane/](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane)
- **Documentation**: [ArduPlane/README.md](ArduPlane/README.md), [QUADPLANE.md](ArduPlane/QUADPLANE.md)
- **User Wiki**: [ardupilot.org/plane/](https://ardupilot.org/plane/index.html)
- **Features**: Fixed-wing control, QuadPlane VTOL, tailsitters, tiltrotors, thermal soaring

#### Rover - Ground and Surface Vehicles
- **Code**: [Rover/](https://github.com/ArduPilot/ardupilot/tree/master/Rover)
- **Documentation**: [Rover/README.md](Rover/README.md)
- **User Wiki**: [ardupilot.org/rover/](https://ardupilot.org/rover/index.html)
- **Features**: Skid-steering, Ackermann steering, boats, sailboats, balance bots

#### ArduSub - Underwater Vehicles
- **Code**: [ArduSub/](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub)
- **Documentation**: [ArduSub/README.md](ArduSub/README.md)
- **User Wiki**: [ardusub.com](http://ardusub.com/)
- **Features**: ROV control, depth hold, attitude stabilization, camera control

#### AntennaTracker - Antenna Tracking Systems
- **Code**: [AntennaTracker/](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker)
- **Documentation**: [AntennaTracker/README.md](AntennaTracker/README.md)
- **User Wiki**: [ardupilot.org/antennatracker/](https://ardupilot.org/antennatracker/index.html)
- **Features**: Automatic antenna pointing for maintaining vehicle communication

#### Blimp - Lighter-Than-Air Vehicles
- **Code**: [Blimp/](https://github.com/ArduPilot/ardupilot/tree/master/Blimp)
- **Documentation**: [Blimp/README.md](Blimp/README.md)
- **Features**: Experimental support for airship and blimp control

## Build System

ArduPilot uses a custom Waf-based build system that supports cross-compilation for numerous hardware platforms.

### Quick Build Commands

```bash
# Configure for a specific board
./waf configure --board=<board_name>

# Build all vehicle types for configured board
./waf build

# Build specific vehicle
./waf copter    # or plane, rover, sub, antennatracker, etc.

# Build for SITL simulation
./waf configure --board=sitl
./waf build
```

### Supported Build Platforms
- Linux (native and cross-compilation)
- macOS (native and cross-compilation)
- Windows (via WSL, Cygwin, or MSYS2)
- Various embedded platforms through custom toolchains

### Build System Documentation
- **Comprehensive Guide**: [BUILD_SYSTEM.md](BUILD_SYSTEM.md)
- **Waf Extensions**: `Tools/ardupilotwaf/README.md`
- **Board Definitions**: `libraries/AP_HAL_ChibiOS/hwdef/`
- **Feature Configuration**: Compile-time feature flags and optimization settings

### Build Targets
- Production firmware for autopilot hardware
- Software-In-The-Loop (SITL) simulation
- Native unit tests
- Replay tool for log analysis
- Examples and test programs

## Testing and Simulation

ArduPilot provides comprehensive testing infrastructure to ensure code quality and safety:

### Software-In-The-Loop (SITL) Simulation
SITL allows running ArduPilot on your development machine with physics simulation:

```bash
# Start SITL for copter
sim_vehicle.py -v ArduCopter

# Start SITL for plane with specific location
sim_vehicle.py -v ArduPlane -L <location>

# Start with Mission Planner
sim_vehicle.py -v ArduCopter --console --map
```

**SITL Features**:
- Physics simulation for multiple vehicle types
- Sensor simulation (GPS, IMU, compass, barometer, etc.)
- No hardware required for development
- Safe testing of experimental features
- Integration with ground station software

### Autotest Framework
Automated testing system in `Tools/autotest/`:
- Comprehensive test scenarios for all vehicle types
- Continuous integration via GitHub Actions
- Mission execution validation
- Failsafe testing
- Mode transition testing

**Running Autotests**:
```bash
# Run all copter tests
Tools/autotest/autotest.py test.ArduCopter

# Run specific test
Tools/autotest/autotest.py test.ArduCopter.Fly.AutoTest
```

### Unit Testing
- C++ unit tests in `libraries/*/tests/`
- Test coverage reporting
- Continuous integration for pull requests

### Hardware-In-The-Loop (HIL) Testing
- Real autopilot hardware with simulated sensors
- Integration testing with actual hardware interfaces

### Test Coverage
See current test status in badges at the top of this README and on [autotest.ardupilot.org](https://autotest.ardupilot.org/).

## Hardware Compatibility

ArduPilot supports a wide range of autopilot hardware through the Hardware Abstraction Layer:

### Autopilot Boards
**100+ supported autopilot boards including**:
- Pixhawk family (Pixhawk 1-6, various vendors)
- Cube family (Orange, Yellow, Purple, Black)
- Holybro Kakute and other F4/F7/H7 flight controllers
- CUAV family (V5+, Nora, X7, etc.)
- mRo family (X2.1, Control Zero, etc.)
- Linux-based boards (Raspberry Pi, BeagleBone, Navio+, etc.)
- Custom and experimental platforms

**Board Definitions**: Hardware configuration files in `libraries/AP_HAL_ChibiOS/hwdef/`

### Supported Peripherals
- **GPS**: u-blox, Trimble, NMEA, DroneCAN GPS, RTK systems
- **Telemetry**: 3DR Radio, RFD900, ESP8266/ESP32, LTE modules
- **Sensors**: Various IMU, compass, barometer, rangefinder, optical flow
- **Cameras**: GoPro, Sony, Canon control, camera triggers
- **Gimbals**: Storm32, Alexmos, Gremsy, servo-based
- **ESCs**: PWM, OneShot, DShot, BLHeli integration, DroneCAN ESCs
- **Lidar**: Lightware, Benewake, Leddar, TeraRanger, RPLidar
- **Companion Computers**: Raspberry Pi, NVIDIA Jetson, Intel NUC

### Communication Protocols
- **MAVLink**: v1 and v2 protocol support
- **DroneCAN/UAVCAN**: CAN bus device support
- **FrSky**: Telemetry and RC protocols
- **MSP**: Betaflight/iNav protocol for OSD and telemetry
- **SBUS, PPM, PWM**: RC input protocols

### Hardware Requirements
- **Minimum**: 32-bit processor, 256KB flash, 64KB RAM (basic features)
- **Recommended**: STM32F7/H7, 1MB+ flash, 256KB+ RAM (full features)
- **Sensors**: IMU (accelerometer + gyroscope), barometer required; GPS, compass recommended

For board-specific information, see `libraries/AP_HAL_ChibiOS/hwdef/<board>/README.md`

## User Support & Discussion Forums ##

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information ##

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://ardupilot.org/dev/>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://discord.com/channels/ardupilot>

## Top Contributors ##

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## Contributing

ArduPilot is an open-source project that thrives on community participation. We welcome contributions of all types!

### Ways to Contribute

#### Code Contributions
- **Pull Requests**: Submit code improvements, bug fixes, and new features
- **Coding Standards**: Follow our [contribution guidelines](https://ardupilot.org/dev/docs/contributing.html)
- **Comprehensive Guide**: See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed instructions

#### Testing and Quality Assurance
- **Beta Testing**: Join our [active beta testing community](https://ardupilot.org/dev/docs/release-procedures.html)
- **Log Analysis**: Help users diagnose issues in the [support forums](https://discuss.ardupilot.org/)
- **Autotest**: Contribute test scenarios to improve coverage

#### Documentation
- **Wiki Improvements**: Enhance user and developer documentation
- **Code Documentation**: Add inline documentation and examples
- **Join Documentation Team**: Chat with [wiki editors on Discord #documentation](https://discord.com/channels/ardupilot)

#### Issue Reporting and Feature Requests
- **Bug Reports**: Report issues on our [GitHub issues list](https://github.com/ArduPilot/ardupilot/issues)
- **Feature Requests**: Propose enhancements with detailed use cases
- **Issue Triage**: Help categorize and prioritize reported issues

#### Community Support
- **Forum Support**: Answer questions on [discuss.ardupilot.org](https://discuss.ardupilot.org/)
- **Discord Community**: Provide real-time help on [Discord](https://ardupilot.org/discord)
- **Tutorial Creation**: Create guides and educational content

#### Getting Started with Contributing
1. Read [CONTRIBUTING.md](CONTRIBUTING.md) for comprehensive guidelines
2. Review the [developer documentation](https://ardupilot.org/dev/)
3. Set up your development environment using `Tools/environment_install/`
4. Start with "good first issue" labels on GitHub
5. Join developer discussions on [Discord](https://discord.com/channels/ardupilot)
6. Contact developers through our [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License ##

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers ##

ArduPilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Willian Galvani](https://github.com/williangalvani):
  - ***Vehicle***: Sub
  - ***Board***: Navigator
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Siddharth Purohit](https://github.com/bugobliterator):
  - ***Subsystem***: CAN, Compass
  - ***Board***: Cube*
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
- [Samuel Tabor](https://github.com/samuelctabor):
  - ***Subsystem***: Soaring/Gliding
- [Henry Wurzburg](https://github.com/Hwurzburg):
  - ***Subsystem***: OSD
  - ***Site***: Wiki
- [Peter Hall](https://github.com/IamPete1):
  - ***Vehicle***: Tailsitters
  - ***Vehicle***: Sailboat
  - ***Subsystem***: Scripting
- [Andy Piper](https://github.com/andyp1per):
  - ***Subsystem***: Crossfire
  - ***Subsystem***: ESC
  - ***Subsystem***: OSD
  - ***Subsystem***: SmartAudio
- [Alessandro Apostoli ](https://github.com/yaapu):
  - ***Subsystem***: Telemetry
  - ***Subsystem***: OSD
- [Rishabh Singh ](https://github.com/rishabsingh3003):
  - ***Subsystem***: Avoidance/Proximity
- [David Bussenschutt ](https://github.com/davidbuzz):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
- [Charles Villard ](https://github.com/Silvanosky):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
