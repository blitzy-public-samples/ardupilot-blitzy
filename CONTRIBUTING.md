# Contributing to ArduPilot

Welcome to the ArduPilot project! We're excited that you're interested in contributing to the world's most advanced open-source autopilot software. ArduPilot powers millions of vehicles worldwide, from multirotors and fixed-wing aircraft to rovers, boats, submarines, and even blimps.

This guide will help you understand how to contribute effectively to this safety-critical codebase while maintaining the high standards that make ArduPilot trusted by hobbyists, researchers, and commercial operators globally.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Project Overview](#project-overview)
- [Getting Started](#getting-started)
- [Development Environment Setup](#development-environment-setup)
- [Code Contribution Workflow](#code-contribution-workflow)
- [Coding Standards](#coding-standards)
- [Documentation Requirements](#documentation-requirements)
- [Testing Requirements](#testing-requirements)
- [Pull Request Process](#pull-request-process)
- [Safety-Critical Code Guidelines](#safety-critical-code-guidelines)
- [Issue Reporting](#issue-reporting)
- [Beta Testing and Releases](#beta-testing-and-releases)
- [Community and Support](#community-and-support)
- [License and Legal](#license-and-legal)

## Code of Conduct

All contributors must adhere to the [ArduPilot Code of Conduct](CODE_OF_CONDUCT.md). Key principles include:

- **Peaceful Use**: ArduPilot aims to enable trusted, autonomous vehicle systems for peaceful benefit
- **No Weaponization**: We do not support or facilitate weaponization of ArduPilot systems
- **No Manned Aircraft**: ArduPilot is NOT certified for manned aircraft or applications where it controls human lives
- **Respectful Collaboration**: We foster an inclusive, harassment-free environment for all contributors

By contributing, you accept and commit to upholding these principles.

## Project Overview

### What is ArduPilot?

ArduPilot is a comprehensive autopilot software suite that provides autonomous control for:

- **ArduCopter**: Multirotors, helicopters, and coaxial copters
- **ArduPlane**: Fixed-wing aircraft and VTOL/QuadPlanes
- **Rover**: Ground vehicles and boats
- **ArduSub**: Underwater ROVs and AUVs
- **AntennaTracker**: Antenna tracking systems
- **Blimp**: Lighter-than-air vehicles

### Architecture

ArduPilot consists of:

- **Vehicle Code**: Vehicle-specific implementations (ArduCopter/, ArduPlane/, Rover/, etc.)
- **Libraries**: 120+ shared libraries for hardware abstraction, sensors, navigation, and control
- **HAL (Hardware Abstraction Layer)**: Platform-independent hardware interfaces
- **Build System**: Waf-based build system supporting multiple boards and configurations
- **Testing Framework**: Comprehensive SITL simulation and autotest infrastructure

### Key Technologies

- **Languages**: Primarily C++ (C++11 standard), with Python for tooling
- **Real-Time Systems**: Hard real-time requirements (typically 400Hz main loop)
- **Safety-Critical**: Extensive failsafe systems and pre-arm checks
- **Sensor Fusion**: Extended Kalman Filter (EKF) for state estimation
- **Communication**: MAVLink protocol, DroneCAN/UAVCAN support

## Getting Started

### Prerequisites

Before contributing, you should have:

- **Basic Knowledge**: Understanding of C++ programming and embedded systems
- **Git Proficiency**: Familiarity with Git workflows (branching, merging, rebasing)
- **Domain Understanding**: Basic knowledge of flight control, navigation, or robotics
- **Hardware** (optional): Access to autopilot hardware for testing, or use SITL simulation

### First Steps for New Contributors

1. **Join the Community**: Introduce yourself on the [discussion forums](https://discuss.ardupilot.org)
2. **Read Documentation**: Review the [developer documentation](https://ardupilot.org/dev/)
3. **Explore the Code**: Browse the codebase to understand its structure
4. **Start Small**: Look for "good first issue" tags on GitHub
5. **Ask Questions**: Don't hesitate to ask on forums or Discord

### Finding Issues to Work On

- **Good First Issues**: GitHub issues labeled `good-first-issue`
- **Documentation**: Help improve inline comments and README files
- **Bug Fixes**: Address reported bugs with clear reproduction steps
- **Feature Requests**: Implement community-requested features
- **Testing**: Expand test coverage in autotest framework

## Development Environment Setup

### Supported Development Platforms

- **Linux**: Ubuntu 20.04 LTS or newer (recommended)
- **macOS**: Recent versions with Xcode Command Line Tools
- **Windows**: WSL2 (Windows Subsystem for Linux) recommended

### Installation Script

ArduPilot provides automated environment setup scripts:

```bash
# Clone the repository
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Initialize submodules
git submodule update --init --recursive

# Run environment setup (Linux/Ubuntu)
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload PATH
. ~/.profile
```

For other platforms, see `Tools/environment_install/` for platform-specific scripts.

### Manual Setup Requirements

Core dependencies include:

- **Python 3**: Version 3.6 or newer
- **Compiler**: GCC 9+ or compatible (arm-none-eabi-gcc for embedded targets)
- **Build Tools**: waf, make, cmake
- **Simulation**: MAVProxy for SITL testing

### Building ArduPilot

#### SITL (Software In The Loop) Build

SITL allows testing without hardware:

```bash
# Build for Copter SITL
./waf configure --board sitl
./waf copter

# Run SITL simulation
sim_vehicle.py -v ArduCopter --console --map
```

#### Hardware Build

Build for specific autopilot boards:

```bash
# Configure for specific board (e.g., Pixhawk4)
./waf configure --board Pixhawk4

# Build vehicle firmware
./waf copter

# Firmware will be in build/Pixhawk4/bin/
```

#### Available Vehicles

```bash
./waf copter        # ArduCopter
./waf plane         # ArduPlane
./waf rover         # Rover
./waf sub           # ArduSub
./waf antennatracker # AntennaTracker
./waf blimp         # Blimp
```

### IDE Configuration

Recommended editors with C++ support:

- **Visual Studio Code**: With C/C++ extension
- **CLion**: Full CMake integration available
- **Eclipse CDT**: Project files can be generated
- **Vim/Emacs**: With appropriate C++ plugins

## Code Contribution Workflow

### Standard GitHub Workflow

ArduPilot follows the standard fork-and-pull-request model:

#### 1. Fork the Repository

- Navigate to https://github.com/ArduPilot/ardupilot
- Click "Fork" to create your copy
- Clone your fork locally:

```bash
git clone https://github.com/YOUR_USERNAME/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

#### 2. Configure Remotes

```bash
# Add upstream remote
git remote add upstream https://github.com/ArduPilot/ardupilot.git

# Verify remotes
git remote -v
```

#### 3. Create a Feature Branch

Always work on a dedicated branch, never on master:

```bash
# Update your master
git checkout master
git pull upstream master

# Create feature branch with descriptive name
git checkout -b feature/descriptive-name
# or
git checkout -b fix/issue-123-description
```

#### Branch Naming Conventions

- `feature/description` - New features
- `fix/issue-number-description` - Bug fixes
- `docs/description` - Documentation updates
- `test/description` - Test additions/improvements
- `refactor/description` - Code refactoring

#### 4. Make Your Changes

- Write clean, well-documented code following [coding standards](#coding-standards)
- Include comprehensive inline documentation
- Add or update tests as needed
- Ensure all tests pass locally

#### 5. Commit Your Changes

Write clear, descriptive commit messages:

```bash
git add <changed-files>
git commit
```

**Commit Message Format**:

```
Component: Brief description (50 chars or less)

More detailed explanation if needed. Wrap at 72 characters.
Explain the problem that this commit solves, and why you chose
this particular solution.

Fixes #issue_number
```

**Examples**:

```
AP_Motors: Fix matrix motor ordering for hexacopter

The motor ordering was incorrect for X configuration hexacopters,
causing instability. This fixes the ordering to match the standard
convention.

Fixes #12345
```

```
AP_InertialSensor: Add support for ICM-45686 IMU

Implements backend driver for InvenSense ICM-45686 with:
- SPI communication at 24MHz
- Configurable sample rate up to 8kHz
- FIFO buffering support

Tested on Pixhawk6X hardware.
```

#### 6. Keep Your Branch Updated

Regularly sync with upstream to avoid conflicts:

```bash
# Fetch upstream changes
git fetch upstream

# Rebase your branch (preferred over merge)
git rebase upstream/master

# If conflicts occur, resolve them and continue
git rebase --continue
```

#### 7. Push to Your Fork

```bash
git push origin feature/your-branch-name

# If you rebased, you may need force push
git push --force-with-lease origin feature/your-branch-name
```

#### 8. Create Pull Request

- Navigate to your fork on GitHub
- Click "Compare & pull request"
- Fill out the PR template completely
- Link related issues using "Fixes #issue_number"

### Multi-Commit Pull Requests

For larger features, multiple logical commits are acceptable:

- Each commit should be self-contained and compilable
- Commit sequence should tell a story
- Avoid "fix typo" or "oops" commits - use interactive rebase to squash
- Final commit series should be clean and reviewable

### Updating Pull Requests

When changes are requested:

```bash
# Make requested changes
# Commit them
git commit -m "Address review feedback"

# Clean up history before final merge (optional)
git rebase -i upstream/master

# Force push updated branch
git push --force-with-lease origin feature/your-branch-name
```

## Coding Standards

ArduPilot maintains strict coding standards for consistency, readability, and safety.

### General Principles

- **Readability First**: Code is read far more than written
- **Explicit Over Implicit**: Be clear about intentions
- **Safety First**: Consider failure modes and edge cases
- **No Warnings**: Code must compile without warnings
- **Portable Code**: Avoid platform-specific code outside HAL

### C++ Style Guidelines

#### Naming Conventions

```cpp
// Classes: PascalCase
class AP_InertialSensor {};
class AC_AttitudeControl {};

// Functions and methods: snake_case
void get_accel_data();
bool perform_calibration();

// Member variables: snake_case with leading underscore for private
class MyClass {
public:
    float public_member;
private:
    float _private_member;
    uint32_t _sample_count;
};

// Constants: UPPER_SNAKE_CASE
#define GRAVITY_MSS 9.80665f
static const float MAX_TILT_ANGLE = 45.0f;

// Parameters: UPPER_SNAKE_CASE with component prefix
AP_GROUPINFO("RATE_P", 0, AC_AttitudeControl, _pid_rate_roll.kP(), 0.15f),

// Enums: PascalCase for type, UPPER_SNAKE_CASE for values
enum class FlightMode {
    STABILIZE = 0,
    ACRO = 1,
    ALT_HOLD = 2,
};
```

#### Code Formatting

```cpp
// Indentation: 4 spaces (no tabs)
void example_function()
{
    if (condition) {
        do_something();
    }
}

// Braces: Opening brace on same line for control structures
if (condition) {
    // code
} else {
    // code
}

// Functions: Opening brace on new line
void my_function()
{
    // code
}

// Line length: Maximum 120 characters (prefer 80-100)

// Pointer/reference alignment: Attached to type
const char* ptr;
Vector3f& accel_vector;

// Spaces around operators
int result = a + b * c;
if (x == y) {
    // code
}

// No space after function names
void function_call(int param);
```

#### Modern C++ Usage

ArduPilot uses C++11 standard. Use modern features appropriately:

```cpp
// Use nullptr instead of NULL
Sensor* sensor = nullptr;

// Use auto for complex types (when type is obvious)
auto imu_instance = AP::ins();

// Range-based for loops (where appropriate)
for (const auto& sensor : sensor_list) {
    sensor.update();
}

// Use override keyword for virtual functions
class Derived : public Base {
    void update() override;
};

// Use constexpr for compile-time constants
static constexpr float GRAVITY = 9.80665f;

// Prefer <cstdint> types for fixed-width integers
uint32_t timestamp;
int16_t raw_sensor_value;
```

#### What to Avoid

```cpp
// ❌ Avoid dynamic memory allocation in flight code
// (Exceptions: initialization only, or with extreme caution)
void bad_example() {
    float* data = new float[100];  // Avoid in flight path
}

// ✅ Use stack or pre-allocated static memory
static float data[100];

// ❌ Avoid exceptions (disabled in build system)
try {
    risky_operation();
} catch (...) {
    // This won't compile
}

// ✅ Use return codes or error flags
if (!risky_operation()) {
    handle_error();
}

// ❌ Avoid C-style casts
float value = (float)int_value;

// ✅ Use C++ casts
float value = static_cast<float>(int_value);
```

### Python Style Guidelines

Python code (tooling, tests) follows PEP 8 with minor modifications:

```python
# Indentation: 4 spaces
def example_function(param1, param2):
    if condition:
        do_something()
    return result

# Function names: snake_case
def calculate_waypoint_distance():
    pass

# Class names: PascalCase
class AutoTestCopter:
    pass

# Constants: UPPER_SNAKE_CASE
MAX_TIMEOUT = 30.0
DEFAULT_ALTITUDE = 10

# Use type hints for new code
def process_telemetry(vehicle: Vehicle, timeout: float = 30.0) -> bool:
    pass
```

### Code Organization

#### File Structure

```cpp
// Header file (.h)
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "LocalDependency.h"

/**
 * @class MyClass
 * @brief Brief description
 */
class MyClass {
public:
    // Public interface
    MyClass();
    void init();
    void update();
    
private:
    // Private implementation
    float _internal_state;
    void _private_helper();
};

// Implementation file (.cpp)
#include "MyClass.h"

MyClass::MyClass()
    : _internal_state(0.0f)
{
}

void MyClass::init()
{
    // Implementation
}
```

#### Include Order

1. Own header file (for .cpp files)
2. Blank line
3. C system headers
4. C++ system headers
5. Blank line
6. Local project headers (AP_*, AC_*, etc.)

```cpp
#include "AP_Example.h"

#include <stdio.h>
#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_Example_Backend.h"
```

## Documentation Requirements

**CRITICAL**: All code contributions must include comprehensive documentation. Pull requests with insufficient documentation will be rejected.

### Documentation Standards

ArduPilot uses Doxygen-style documentation for C++ code and Markdown for guides.

#### Function Documentation (Mandatory)

Every public function and method must have complete Doxygen documentation:

```cpp
/**
 * @brief Calculate desired attitude from pilot input and current state
 * 
 * @details This function implements rate-controlled stabilize mode,
 *          converting pilot stick inputs to desired attitude quaternions
 *          while respecting configured limits and expo curves.
 * 
 * @param[in]  pilot_roll_cd   Pilot roll input in centidegrees (-4500 to 4500)
 * @param[in]  pilot_pitch_cd  Pilot pitch input in centidegrees (-4500 to 4500)
 * @param[in]  pilot_yaw_rate  Pilot yaw rate in deg/s
 * @param[out] target_attitude Desired vehicle attitude quaternion
 * 
 * @return true if calculation successful, false if limits exceeded
 * 
 * @note This is called at main loop rate (typically 400Hz)
 * @warning Modifying rate limits can affect vehicle stability
 */
bool calculate_attitude_target(int16_t pilot_roll_cd, int16_t pilot_pitch_cd, 
                              float pilot_yaw_rate, Quaternion &target_attitude);
```

#### Class Documentation (Mandatory)

```cpp
/**
 * @class AP_InertialSensor
 * @brief Manages IMU (accelerometer/gyroscope) hardware and data processing
 * 
 * @details Provides unified interface for multiple IMU backends, handles
 *          sensor calibration, sample rate management, and FIFO processing.
 *          Supports multiple concurrent IMU instances for redundancy.
 * 
 * Singleton access: AP::ins()
 * 
 * Example usage:
 * @code
 * AP_InertialSensor &ins = AP::ins();
 * ins.init(400);  // 400Hz sample rate
 * ins.wait_for_sample();
 * Vector3f accel = ins.get_accel();
 * @endcode
 */
class AP_InertialSensor {
    // class definition
};
```

#### Complex Logic Documentation (Mandatory)

Any non-obvious code must be explained:

```cpp
void update_sensor_fusion()
{
    // Calculate innovation (difference between predicted and measured)
    // Using NED (North-East-Down) coordinate frame convention
    Vector3f innovation = measured_position - predicted_position;
    
    // Apply Kalman gain only if innovation is within expected bounds
    // This prevents outlier measurements from corrupting the estimate
    if (innovation.length() < innovation_gate_threshold) {
        // Kalman update: state = state + gain * innovation
        state += kalman_gain * innovation;
    }
}
```

#### Safety-Critical Code Documentation (MANDATORY)

Safety-critical paths require extensive documentation:

```cpp
/**
 * @brief Handle critical battery failsafe condition
 * 
 * @safety CRITICAL: This function determines emergency actions for battery depletion
 * 
 * @failsafe_trigger Battery voltage below BATT_LOW_VOLT or remaining capacity below BATT_LOW_MAH
 * @failsafe_actions 
 *   - Level 1: Warning to pilot, reduce throttle limits
 *   - Level 2: Force RTL (Return to Launch)
 *   - Level 3: Emergency land at current location
 * @failsafe_priority Overrides pilot input, but can be overridden by GCS in extreme cases
 * 
 * @thread_safety Called from main thread only, not interrupt-safe
 * 
 * @param[in] battery_level Current battery percentage (0-100)
 * @param[in] voltage_mv Battery voltage in millivolts
 */
void handle_battery_failsafe(uint8_t battery_level, uint32_t voltage_mv);
```

### Parameter Documentation (Mandatory)

All parameters must be documented in their definition:

```cpp
// @Param: RATE_P
// @DisplayName: Roll/Pitch rate controller P gain
// @Description: Proportional gain for roll and pitch rate control
// @Range: 0.08 0.35
// @Increment: 0.005
// @User: Standard
AP_GROUPINFO("RATE_P", 0, AC_AttitudeControl, _pid_rate_roll.kP(), 0.15f),
```

### Module-Level Documentation (Required for New Modules)

Every library should have a comprehensive README.md:

```markdown
# AP_Motors Library

## Overview
The AP_Motors library provides a unified interface for motor control across different 
vehicle types, handling motor mixing, throttle curves, and output limiting.

## Architecture
[Include Mermaid diagram showing class hierarchy]

## Key Components
- `AP_Motors_Class`: Base class defining the interface
- `AP_MotorsMatrix`: Implementation for multicopters
- `AP_MotorsTri`: Tricopter-specific implementation

## Integration Guide
### Adding a New Frame Type
1. Define frame type in AP_Motors_Class.h
2. Implement motor mixing in setup_motors()
3. Add frame to init switch statement

## Configuration Parameters
- MOT_SPIN_MIN: Minimum throttle for motors when armed
- MOT_SPIN_MAX: Point at which throttle saturates

## Testing
Use motor test feature in ground station to verify motor order...
```

### Documentation Quality Requirements

- **Units**: Always specify units for numeric parameters (e.g., meters, degrees, milliseconds)
- **Ranges**: Document valid ranges for parameters
- **Thread Safety**: Note if functions are thread-safe or require locks
- **Timing**: Document real-time constraints and execution frequency
- **Side Effects**: Clearly document any state modifications
- **Error Conditions**: Document all possible error returns

### Documentation Review Checklist

Before submitting, verify:

- [ ] All public functions have Doxygen headers
- [ ] All parameters include units and valid ranges
- [ ] Complex algorithms are explained with comments
- [ ] Safety-critical code has comprehensive safety documentation
- [ ] All new parameters have @Param documentation
- [ ] Any new library includes README.md
- [ ] Code examples are correct and compilable

**Note**: Reviewers will specifically check documentation completeness. Incomplete documentation is grounds for PR rejection.

## Testing Requirements

ArduPilot has rigorous testing requirements. All code changes must be thoroughly tested before submission.

### Testing Hierarchy

1. **Unit Tests**: Test individual functions and classes in isolation
2. **SITL Tests**: Test complete vehicle behavior in simulation
3. **Hardware Tests**: Verify on actual autopilot hardware
4. **Flight Tests**: Real-world validation (when applicable)

### Unit Testing

#### Writing Unit Tests

Unit tests are located in `libraries/*/tests/` directories:

```cpp
// libraries/AP_Math/tests/test_vector3.cpp
#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>

TEST(Vector3Test, Length)
{
    Vector3f v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

TEST(Vector3Test, Normalize)
{
    Vector3f v(3.0f, 4.0f, 0.0f);
    v.normalize();
    EXPECT_FLOAT_EQ(v.length(), 1.0f);
}
```

#### Running Unit Tests

```bash
# Configure for SITL with unit tests
./waf configure --board sitl --enable-tests

# Build and run tests
./waf tests

# Run specific test
./build/sitl/tests/test_vector3
```

### SITL (Software-In-The-Loop) Testing

SITL provides complete vehicle simulation without hardware:

#### Running SITL Tests

```bash
# Start SITL with default parameters
sim_vehicle.py -v ArduCopter --console --map

# Run the full autotest suite
Tools/autotest/autotest.py run.ArduCopter

# Run specific test
Tools/autotest/autotest.py test.ArduCopter.ArmFeatures
```

#### Writing SITL Tests

Add tests to `Tools/autotest/arducopter.py` (or appropriate vehicle file):

```python
def MyNewTest(self):
    """Test description"""
    # Takeoff to 10 meters
    self.takeoff(10, mode="GUIDED")
    
    # Change mode
    self.change_mode("LOITER")
    
    # Verify position hold
    self.wait_groundspeed(0, 2, timeout=10)
    
    # Land and disarm
    self.land_and_disarm()

# Add to test list
tests1 = [
    # ... existing tests
    ("MyNewTest", "Test new feature"),
]
```

#### SITL Test Best Practices

- Test should be repeatable and deterministic
- Use appropriate timeouts (balance thoroughness vs test duration)
- Clean up state between tests
- Verify expected behavior, don't just check for crashes
- Test both success and failure paths

### Hardware Testing

Before submitting hardware-related changes:

#### Minimal Hardware Testing

1. **Boot Test**: Firmware boots without errors
2. **Sensor Detection**: Required sensors are detected
3. **Calibration**: Calibration procedures complete successfully
4. **Arming**: Vehicle arms with no pre-arm failures
5. **Basic Control**: Motors/servos respond to inputs appropriately

#### Hardware Test Logging

```bash
# Enable detailed logging
param set LOG_DISARMED 1
param set LOG_REPLAY 1

# Perform tests, then download logs
# Analyze with https://logs.ardupilot.org/
```

### Testing Requirements by Change Type

| Change Type | Required Tests |
|-------------|----------------|
| **Bug Fix** | Unit test demonstrating bug + fix, SITL regression test |
| **New Feature** | Comprehensive unit tests, SITL integration tests, hardware validation |
| **Library Change** | Unit tests for library, SITL tests for affected vehicles |
| **HAL Change** | Tests on all affected platforms, hardware validation |
| **Safety-Critical** | Exhaustive testing, multiple reviewers, extended beta period |
| **Documentation Only** | Build verification, link checking |

### Test Coverage Expectations

- **New Code**: Aim for >80% test coverage
- **Modified Code**: Ensure existing tests still pass, add tests for new scenarios
- **Safety-Critical Code**: 100% coverage of all code paths

### Continuous Integration (CI)

All pull requests automatically run:

- **Build Tests**: Compilation for all supported boards
- **Unit Tests**: Complete unit test suite
- **Autotest**: Core SITL test scenarios
- **Style Checks**: Code formatting and standards compliance

**Pull requests must pass all CI checks before merge.**

### Regression Testing

When fixing bugs:

1. **Reproduce**: Demonstrate the bug in a test
2. **Fix**: Implement the fix
3. **Verify**: Confirm the test now passes
4. **Protect**: Ensure test remains in suite to prevent regression

Example:

```python
def TestBugFix12345(self):
    """Verify fix for issue #12345 - altitude loss in AUTO mode"""
    self.takeoff(10, mode="GUIDED")
    self.change_mode("AUTO")
    
    # Load mission with waypoints
    self.load_mission("mission.txt")
    
    # Verify altitude maintained within tolerance
    self.wait_altitude(9, 11, timeout=30)
    
    # Issue #12345: altitude was dropping below 8m
    # This test ensures it stays above 9m
```

### Pre-Submission Testing Checklist

Before submitting your PR, verify:

- [ ] Code compiles without warnings for primary targets
- [ ] All unit tests pass
- [ ] SITL tests pass for affected vehicles
- [ ] No regressions in existing functionality
- [ ] New functionality has corresponding tests
- [ ] Tested on actual hardware (if applicable)
- [ ] Logs reviewed for errors or warnings
- [ ] Performance impact assessed (timing, memory)

### Performance Testing

For performance-sensitive changes:

```bash
# Use SITL replay to analyze timing
Tools/Replay/Replay <logfile>

# Check scheduler timing
# Look for slow loops or overruns in logs
```

Monitor:
- **Loop Time**: Main loop should maintain target rate (typically 400Hz)
- **CPU Load**: Should remain below 80% under normal conditions
- **Memory Usage**: No unbounded allocations
- **Latency**: Control loop latency should be minimal

## Pull Request Process

### Before Opening a Pull Request

1. **Self-Review**: Review your own code critically
2. **Test Thoroughly**: Complete all required testing
3. **Documentation**: Ensure all documentation is complete
4. **Clean History**: Squash or rebase commits into logical units
5. **Update from Master**: Rebase on latest upstream/master

### Pull Request Template

When creating a PR, provide complete information:

```markdown
## Description
Brief summary of changes and motivation

## Related Issues
Fixes #1234
Related to #5678

## Type of Change
- [ ] Bug fix (non-breaking change fixing an issue)
- [ ] New feature (non-breaking change adding functionality)
- [ ] Breaking change (fix or feature causing existing functionality to change)
- [ ] Documentation update

## Testing
Describe testing performed:
- [ ] Unit tests added/updated
- [ ] SITL tests pass
- [ ] Hardware tested on: [board names]
- [ ] Flight tested: [conditions, duration]

## Checklist
- [ ] Code follows project style guidelines
- [ ] Self-review completed
- [ ] Comments added for complex code
- [ ] Documentation updated
- [ ] No new warnings generated
- [ ] Tests added/updated
- [ ] All tests pass
- [ ] Rebased on current master

## Additional Context
Any additional information, screenshots, logs, or context
```

### Review Process

#### Review Timeline

- **Initial Response**: Typically within 1-3 days
- **Full Review**: May take 1-2 weeks depending on complexity
- **Iterations**: Multiple rounds of review are normal

#### What Reviewers Look For

1. **Correctness**: Does it solve the problem correctly?
2. **Safety**: Are there any safety implications?
3. **Performance**: Impact on timing, memory, CPU usage
4. **Style**: Adherence to coding standards
5. **Documentation**: Completeness of documentation
6. **Testing**: Adequate test coverage
7. **Maintainability**: Is the code clear and maintainable?
8. **Compatibility**: Does it break existing functionality?

#### Responding to Review Feedback

- **Be Responsive**: Address feedback promptly
- **Be Professional**: Accept criticism gracefully
- **Ask Questions**: If feedback is unclear, ask for clarification
- **Explain Decisions**: Justify your approach when needed
- **Make Changes**: Update code based on feedback
- **Re-request Review**: After addressing feedback, request another review

#### Common Review Issues

**Documentation Issues**:
- Missing Doxygen headers
- Unclear parameter descriptions
- No units specified
- Missing safety annotations

**Code Quality Issues**:
- Inconsistent style
- Overly complex functions
- Missing error handling
- Resource leaks

**Testing Issues**:
- Insufficient test coverage
- Tests not verifying correct behavior
- Missing edge case tests

### Merge Criteria

A PR will be merged when:

1. **All CI checks pass** (build, tests, style)
2. **At least one maintainer approval** (more for critical changes)
3. **All review comments addressed**
4. **Documentation complete**
5. **Tests adequate**
6. **No merge conflicts** with master

### After Merge

- **Monitor**: Watch for any issues reported after merge
- **Follow-up**: Be available for questions or bug reports
- **Beta Testing**: Participate in beta testing of your feature

## Safety-Critical Code Guidelines

ArduPilot controls vehicles that can cause injury or property damage. Special care is required for safety-critical code.

### Safety-Critical Code Areas

The following areas require extra scrutiny and testing:

#### Flight Control

- **Attitude Control** (`AC_AttitudeControl/`, `AP_AHRS/`)
- **Position Control** (`AC_PosControl/`, `AC_WPNav/`)
- **Motor Mixing** (`AP_Motors/`)
- **Mode Transitions** (`mode*.cpp` files)

#### Failsafe Systems

- **Battery Failsafes** (`failsafe.cpp`, battery handling)
- **RC Loss Failsafes** (`radio.cpp`, failsafe logic)
- **GPS/EKF Failsafes** (`ekf_check.cpp`, `failsafe.cpp`)
- **Geofencing** (`AC_Fence/`, `fence.cpp`)
- **Return to Launch** (RTL mode implementations)

#### Sensor Processing

- **IMU Processing** (`AP_InertialSensor/`)
- **GPS Processing** (`AP_GPS/`)
- **EKF State Estimation** (`AP_NavEKF2/`, `AP_NavEKF3/`)
- **Sensor Health Monitoring** (health check functions)

#### Arming/Disarming

- **Pre-arm Checks** (`AP_Arming/`)
- **Emergency Disarm** (crash detection, failsafe disarm)
- **Motor Interlock** (motor safety, emergency stop)

### Safety-Critical Coding Requirements

#### 1. Defensive Programming

```cpp
// ✅ Always validate inputs
bool set_target_altitude(float altitude_m)
{
    // Validate range
    if (altitude_m < MIN_ALTITUDE || altitude_m > MAX_ALTITUDE) {
        return false;
    }
    
    // Check for NaN/Inf
    if (!isfinite(altitude_m)) {
        return false;
    }
    
    _target_altitude = altitude_m;
    return true;
}

// ✅ Check for division by zero
float calculate_rate(float distance, float time)
{
    if (is_zero(time)) {
        return 0.0f;  // Safe default
    }
    return distance / time;
}

// ✅ Array bounds checking
float get_sensor_value(uint8_t index)
{
    if (index >= NUM_SENSORS) {
        // Log error, return safe default
        return 0.0f;
    }
    return _sensor_data[index];
}
```

#### 2. Fail-Safe Defaults

```cpp
// Always initialize to safe values
class AC_AttitudeControl {
public:
    AC_AttitudeControl() :
        _motors_enabled(false),  // ✅ Disabled by default
        _max_angle_deg(45.0f),   // ✅ Conservative limit
        _emergency_stop(false)
    {
    }
};

// Provide safe fallback behavior
void update_motor_output()
{
    if (!_motors_enabled || _emergency_stop || !_output_valid) {
        // ✅ Safe state: disable outputs
        output_motor_zero();
        return;
    }
    
    // Normal operation
    output_motor_calculated();
}
```

#### 3. Explicit State Management

```cpp
// Make state transitions explicit and logged
void set_mode(FlightMode new_mode)
{
    FlightMode old_mode = _current_mode;
    
    // Validate transition
    if (!mode_transition_allowed(old_mode, new_mode)) {
        gcs().send_text(MAV_SEVERITY_WARNING, 
                       "Mode change to %s rejected", 
                       mode_name(new_mode));
        return;
    }
    
    // Perform transition
    _current_mode = new_mode;
    
    // Log transition
    AP::logger().Write_Mode(_current_mode);
    gcs().send_text(MAV_SEVERITY_INFO, 
                   "Mode changed: %s -> %s",
                   mode_name(old_mode), 
                   mode_name(new_mode));
}
```

#### 4. Comprehensive Error Handling

```cpp
// Check all operations that can fail
bool initialize_sensor()
{
    if (!_sensor.probe()) {
        // Log detailed error
        DEV_PRINTF("Sensor probe failed on bus %u address 0x%02x\n",
                   _bus, _address);
        return false;
    }
    
    if (!_sensor.configure()) {
        DEV_PRINTF("Sensor configuration failed\n");
        return false;
    }
    
    if (!_sensor.self_test()) {
        DEV_PRINTF("Sensor self-test failed\n");
        return false;
    }
    
    return true;
}
```

#### 5. Thread Safety

```cpp
class SensorData {
private:
    HAL_Semaphore _sem;  // Protects shared data
    Vector3f _latest_sample;
    
public:
    // ✅ Protect shared data access
    Vector3f get_sample()
    {
        WITH_SEMAPHORE(_sem);
        return _latest_sample;
    }
    
    void update_sample(const Vector3f& sample)
    {
        WITH_SEMAPHORE(_sem);
        _latest_sample = sample;
    }
};
```

### Safety Review Requirements

Changes to safety-critical code require:

1. **Multiple Reviewers**: At least 2 experienced reviewers
2. **Extended Testing**: More comprehensive test requirements
3. **Hardware Testing**: Mandatory testing on actual hardware
4. **Beta Period**: Extended beta testing before release
5. **Documentation**: Extensive safety documentation
6. **Risk Assessment**: Documented failure mode analysis

### Safety Testing Checklist

For safety-critical changes:

- [ ] All error paths tested
- [ ] Failure modes identified and handled
- [ ] Resource limits verified (stack, heap, timing)
- [ ] Sensor failure scenarios tested
- [ ] Communication loss scenarios tested
- [ ] Multiple flight tests in various conditions
- [ ] Log analysis for any anomalies
- [ ] Recovery procedures verified

### Prohibited Practices in Safety-Critical Code

**Never do this in safety-critical code:**

```cpp
// ❌ Unchecked pointer dereference
void bad_example(Sensor* sensor)
{
    float value = sensor->read();  // Could crash if null
}

// ✅ Always check pointers
void good_example(Sensor* sensor)
{
    if (sensor == nullptr) {
        return;
    }
    float value = sensor->read();
}

// ❌ Unbounded loops
while (waiting_for_sensor()) {
    // Could hang forever
}

// ✅ Timeout on waits
uint32_t start_ms = AP_HAL::millis();
while (waiting_for_sensor()) {
    if (AP_HAL::millis() - start_ms > TIMEOUT_MS) {
        break;  // Timeout protection
    }
}

// ❌ Ignoring return codes
sensor.read_data();  // What if it failed?

// ✅ Check return codes
if (!sensor.read_data()) {
    handle_error();
}
```

## Issue Reporting

Effective bug reports help improve ArduPilot for everyone.

### Before Reporting an Issue

1. **Search Existing Issues**: Check if it's already reported
2. **Check Documentation**: Verify it's not expected behavior
3. **Test Latest Version**: Reproduce on latest stable or beta
4. **Gather Information**: Collect logs, parameters, and system details

### Bug Report Template

Create issues at https://github.com/ArduPilot/ardupilot/issues with:

```markdown
## Bug Description
Clear and concise description of the bug

## Expected Behavior
What should happen

## Actual Behavior
What actually happens

## Steps to Reproduce
1. Configure vehicle with...
2. Arm and takeoff to...
3. Switch to mode...
4. Observe...

## System Information
- **Vehicle Type**: Copter/Plane/Rover/Sub
- **Firmware Version**: 4.x.x (Stable/Beta)
- **Autopilot Hardware**: Pixhawk 4, Cube Orange, etc.
- **Sensors**: GPS model, IMU, etc.
- **Companion Computer**: Raspberry Pi, etc. (if applicable)

## Log Files
Link to dataflash log: https://logs.ardupilot.org/...

## Additional Context
Screenshots, videos, or other relevant information
```

### Getting Logs

Dataflash logs are essential for diagnosing issues:

```bash
# Enable logging while disarmed (for pre-arm issues)
param set LOG_DISARMED 1

# After the issue occurs, download .bin log files
# Upload to https://logs.ardupilot.org/ for analysis
```

### Issue Triage

Issues are labeled by maintainers:

- **bug**: Confirmed defects
- **enhancement**: Feature requests
- **good-first-issue**: Suitable for new contributors
- **safety-critical**: Requires immediate attention
- **needs-testing**: Requires verification
- **documentation**: Documentation improvements

### Feature Requests

When requesting features:

- **Use Case**: Explain the problem it solves
- **Implementation Ideas**: Suggest approaches (optional)
- **Alternative Solutions**: What are you doing now?
- **Community Interest**: Is this broadly useful?

### Reporting Security Issues

**Do not open public issues for security vulnerabilities.**

Email security concerns to: security@ardupilot.org

Security issues include:
- Remote code execution vulnerabilities
- Authentication bypasses
- Denial of service attacks
- Unauthorized control of vehicle

## Beta Testing and Releases

### Release Cycle

ArduPilot follows a regular release cadence:

- **Stable Releases**: Every 3-4 months (e.g., 4.3.0, 4.4.0)
- **Point Releases**: Bug fixes for stable (e.g., 4.3.1, 4.3.2)
- **Beta Releases**: Pre-release testing (e.g., 4.4.0-beta1)
- **Development Branch**: Latest changes (master branch)

### Beta Testing Program

Beta testing helps identify issues before stable release:

#### Joining Beta Testing

1. **Install Beta Firmware**: Through ground station (Mission Planner, QGroundControl)
2. **Test Thoroughly**: Fly in various conditions
3. **Report Issues**: Submit detailed bug reports
4. **Provide Feedback**: Share experience on forums

#### Beta Testing Best Practices

- **Test Incrementally**: Don't immediately test all new features
- **Keep Stable Backup**: Have ability to revert to stable
- **Safe Environment**: Test in safe locations
- **Document Configuration**: Note all parameter changes
- **Share Logs**: Upload logs even for successful flights

#### What to Test

- **New Features**: Specifically test new functionality
- **Existing Features**: Verify no regressions
- **Edge Cases**: Test unusual configurations or conditions
- **Your Use Case**: Test your specific mission profiles

### Release Process for Contributors

If your contribution makes it into a release:

1. **Beta Period**: Code enters beta testing (2-4 weeks)
2. **Issue Resolution**: Critical issues fixed during beta
3. **Release Candidate**: Final testing phase
4. **Stable Release**: Included in stable firmware
5. **Announcement**: Mentioned in release notes

### Release Notes

Significant contributions are acknowledged in release notes:

- Link to your GitHub profile
- Description of feature/fix
- Thank you to contributors

## Community and Support

### Communication Channels

#### Discussion Forums

Primary community hub: https://discuss.ardupilot.org

- **Help and Support**: User questions and troubleshooting
- **Development**: Technical discussions, architecture decisions
- **Vehicle Categories**: Copter, Plane, Rover, Sub specific forums
- **Blog**: Official announcements and articles

#### Real-Time Chat

**Discord**: https://ardupilot.org/discord

- Development discussions
- Quick questions
- Community interaction
- Voice channels for meetings

#### Developer Calls

- **Weekly Development Call**: Wednesdays (check Discord for time)
- **Monthly Public Call**: First Tuesday of month
- Open to all contributors

### Getting Help

#### For Users

- **Documentation**: https://ardupilot.org/
- **Wiki**: https://ardupilot.org/copter/, /plane/, /rover/, /dev/
- **Discussion Forums**: Post detailed questions
- **Discord**: Quick questions and community help

#### For Developers

- **Developer Documentation**: https://ardupilot.org/dev/
- **Developer Forum**: Technical discussions
- **Discord #developers**: Real-time help
- **Code Comments**: Extensive inline documentation
- **Weekly Calls**: Join developer meetings

### Building Community

#### Ways to Contribute Beyond Code

- **Documentation**: Improve guides and tutorials
- **Support**: Answer questions on forums
- **Testing**: Beta test new releases
- **Translation**: Translate documentation
- **Blog Posts**: Share your projects and experiences
- **Videos**: Create tutorial videos
- **Presentations**: Speak at conferences or meetups

#### Recognition

Active contributors may become:

- **Trusted Committer**: Direct commit access for specific areas
- **Maintainer**: Responsible for specific libraries or vehicle types
- **Development Team Member**: Core team membership

Recognition is based on:
- Quality of contributions
- Consistency over time
- Community engagement
- Domain expertise
- Responsibility and reliability

### Community Guidelines

- **Be Patient**: Contributors volunteer their time
- **Be Respectful**: Treat all community members with respect
- **Be Constructive**: Offer solutions, not just criticism
- **Be Professional**: Maintain professional communication
- **Share Knowledge**: Help others learn and grow

### Resources for Learning

#### Documentation

- **User Documentation**: https://ardupilot.org/copter/ (plane/rover/sub)
- **Developer Documentation**: https://ardupilot.org/dev/
- **Code API Documentation**: https://ardupilot.org/dev/docs/apmcopter/

#### Learning Resources

- **ArduPilot Methodic Configurator**: Systematic setup tool
- **Video Tutorials**: YouTube channel and community videos
- **Example Code**: `/libraries/*/examples/` directories
- **SITL Tutorials**: Software simulation guides

#### Recommended Background Knowledge

- **Control Theory**: PID controllers, state machines
- **Linear Algebra**: Vectors, matrices, quaternions
- **Coordinate Systems**: NED frames, body frames
- **Communication Protocols**: MAVLink, CAN, serial protocols
- **Embedded Systems**: Real-time constraints, resource limits

## License and Legal

### Software License

ArduPilot is licensed under **GNU General Public License v3.0 (GPLv3)**.

#### Key GPLv3 Requirements

- **Source Code**: Modified versions must provide source code
- **Same License**: Derivative works must also be GPLv3
- **Patent Grant**: Contributors grant patent rights
- **No Warranty**: Software provided "as-is" without warranty

#### What This Means for Contributors

By contributing to ArduPilot:

1. **You retain copyright** to your contributions
2. **You grant license** for ArduPilot to use your code under GPLv3
3. **Your code becomes GPLv3**: Cannot be relicensed to proprietary
4. **Attribution**: Your contributions will be credited

#### Developer Certificate of Origin (DCO)

Contributors certify that:

- Code is your original work, or
- You have rights to submit it under GPLv3, and
- You understand it will be publicly available under GPLv3

### Legal Compliance

#### Export Control

Some countries have export control regulations for autopilot software. Users are responsible for compliance with their local laws.

#### Patent Rights

By contributing, you affirm:

- You have rights to contribute the code
- No patent claims prevent its use
- You grant patent license for your contributions

#### Trademark

"ArduPilot" is a trademark. Use in compliance with project guidelines.

### Third-Party Code

When incorporating third-party code:

#### License Compatibility

- **Compatible**: MIT, BSD, Apache 2.0 (can integrate)
- **Incompatible**: Proprietary, GPL-incompatible licenses

#### Attribution Requirements

1. **Preserve License**: Include original license text
2. **Credit Authors**: Maintain copyright notices
3. **Document Source**: Note origin in file header

Example:

```cpp
/*
 * Original code from ProjectX
 * Copyright (C) 20XX Author Name
 * Licensed under MIT License
 * 
 * Modified for ArduPilot integration
 */
```

### Commercial Use

GPLv3 allows commercial use:

- **Hardware Sales**: Sell vehicles with ArduPilot installed
- **Services**: Provide commercial services using ArduPilot
- **Requirement**: Provide source code to customers
- **Modifications**: Share modifications if distributing firmware

### Prohibited Uses

Per Code of Conduct:

- **Weaponization**: Do not contribute to weaponized systems
- **Manned Aircraft**: Not for manned aviation (not certified)
- **Human Control**: Not for systems directly controlling human lives

Violations may result in removal from project and community.

---

## Summary

Contributing to ArduPilot is rewarding and helps advance autonomous vehicle technology worldwide. By following these guidelines, you help maintain the high quality and safety standards that make ArduPilot trusted globally.

### Quick Checklist for New Contributors

1. ✅ Read and accept [Code of Conduct](CODE_OF_CONDUCT.md)
2. ✅ Set up development environment
3. ✅ Build and run SITL simulation
4. ✅ Find an issue to work on or propose a feature
5. ✅ Fork repository and create feature branch
6. ✅ Write code following style guidelines
7. ✅ Add comprehensive documentation
8. ✅ Write and run tests
9. ✅ Submit pull request with complete information
10. ✅ Respond to review feedback

### Getting Started Now

- **Join Discord**: https://ardupilot.org/discord
- **Read Dev Docs**: https://ardupilot.org/dev/
- **Browse Issues**: https://github.com/ArduPilot/ardupilot/issues
- **Ask Questions**: https://discuss.ardupilot.org/c/development

### Thank You!

Thank you for your interest in contributing to ArduPilot. Every contribution, whether code, documentation, testing, or community support, helps make autonomous vehicles safer and more capable.

We look forward to your contributions!

---

**ArduPilot Development Team**

For questions about contributing, contact: ardupilot.devel@gmail.com

