# ArduRover Obstacle Avoidance System

![Status](https://img.shields.io/badge/status-production-green)
![Vehicle](https://img.shields.io/badge/vehicle-rover-blue)

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Path Planning Algorithms](#path-planning-algorithms)
  - [BendyRuler Algorithm](#bendyruler-algorithm)
  - [Dijkstra's Algorithm](#dijkstras-algorithm)
  - [Combined Mode](#combined-mode)
- [Sensor Integration](#sensor-integration)
- [Integration with Navigation](#integration-with-navigation)
- [Configuration Parameters](#configuration-parameters)
- [Tuning and Optimization](#tuning-and-optimization)
- [Operational Modes](#operational-modes)
- [Error Handling and Recovery](#error-handling-and-recovery)
- [Performance Considerations](#performance-considerations)
- [Testing and Validation](#testing-and-validation)

## Overview

The ArduRover obstacle avoidance system provides real-time path planning and collision prevention capabilities for ground vehicles. The system integrates multiple sensing modalities with sophisticated path planning algorithms to enable safe autonomous navigation in complex environments.

**Source Files**: 
- `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp` - Main path planner
- `/libraries/AC_Avoidance/AP_OAPathPlanner.h` - Path planner interface
- `/libraries/AR_WPNav/AR_WPNav_OA.cpp` - Rover waypoint navigation with OA
- `/libraries/AR_WPNav/AR_WPNav_OA.h` - Rover OA interface
- `/libraries/AC_Avoidance/AP_OABendyRuler.cpp` - BendyRuler algorithm
- `/libraries/AC_Avoidance/AP_OADijkstra.cpp` - Dijkstra's algorithm

### Key Features

- **Multiple Path Planning Algorithms**: BendyRuler for dynamic obstacles, Dijkstra for static fence-based planning
- **Multi-Threaded Architecture**: Background thread processes avoidance at 1Hz without blocking main control loop
- **Sensor Fusion**: Integrates proximity sensors, fence boundaries, and dynamic object database
- **Seamless Integration**: Works with existing waypoint navigation and mission systems
- **Real-Time Performance**: Provides alternative paths within 3 seconds or stops vehicle safely

### Supported Obstacle Types

1. **Dynamic Obstacles**: Detected by proximity sensors (range finders, lidars, radar)
2. **Static Fences**: Polygon inclusion/exclusion zones and circular boundaries
3. **Terrain Features**: Height-based obstacles (with vertical avoidance enabled)
4. **Moving Objects**: Tracked via object database with velocity estimates

## System Architecture

The obstacle avoidance system consists of three primary layers working together:

```mermaid
graph TB
    subgraph "Sensor Layer"
        PROX[Proximity Sensors]
        FENCE[Fence Boundaries]
        OADB[Object Database]
    end
    
    subgraph "Path Planning Layer"
        PLANNER[AP_OAPathPlanner]
        BENDY[BendyRuler Algorithm]
        DIJK[Dijkstra Algorithm]
    end
    
    subgraph "Navigation Layer"
        WPNAV[AR_WPNav_OA]
        ATC[Attitude Controller]
        MOTOR[Motor Output]
    end
    
    PROX --> BENDY
    FENCE --> DIJK
    OADB --> BENDY
    OADB --> DIJK
    
    BENDY --> PLANNER
    DIJK --> PLANNER
    
    PLANNER --> WPNAV
    WPNAV --> ATC
    ATC --> MOTOR
    
    style PLANNER fill:#f9f,stroke:#333,stroke-width:2px
    style WPNAV fill:#9ff,stroke:#333,stroke-width:2px
```

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.h:19`

### Component Responsibilities

#### AP_OAPathPlanner

**Purpose**: Central coordinator for all obstacle avoidance path planning

**Key Responsibilities**:
- Manages avoidance algorithm lifecycle (initialization, threading)
- Coordinates between BendyRuler and Dijkstra algorithms
- Provides unified `mission_avoidance()` interface to navigation layer
- Maintains object database for dynamic obstacle tracking
- Runs background thread for non-blocking path computation

**Thread Safety**: Uses `HAL_Semaphore _rsem` to protect shared data between main thread and avoidance thread

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:78-121`

#### AR_WPNav_OA

**Purpose**: Rover-specific waypoint navigation with obstacle avoidance integration

**Key Responsibilities**:
- Calls path planner at regular intervals during navigation
- Switches between normal and OA-adjusted destinations
- Maintains backup of original waypoints for recovery
- Stops vehicle safely if path planning fails
- Provides OA-adjusted bearing and distance for control

**State Management**: Tracks `_oa_active` flag to distinguish normal vs avoidance navigation

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:26-150`

### Data Flow

The obstacle avoidance system operates through a request-response pattern:

```mermaid
sequenceDiagram
    participant NAV as AR_WPNav_OA
    participant PLAN as AP_OAPathPlanner
    participant THREAD as Avoidance Thread
    participant ALG as Algorithm (Bendy/Dijk)
    
    NAV->>PLAN: mission_avoidance(current, origin, dest)
    Note over PLAN: Lock semaphore
    PLAN->>PLAN: Store request
    PLAN->>NAV: Return cached result or OA_PROCESSING
    Note over PLAN: Unlock semaphore
    
    loop Every 1000ms
        THREAD->>PLAN: Read request
        THREAD->>ALG: update(current, dest)
        ALG->>ALG: Search for path
        ALG-->>THREAD: Return new origin/dest
        Note over THREAD: Lock semaphore
        THREAD->>PLAN: Store result
        Note over THREAD: Unlock semaphore
    end
    
    NAV->>PLAN: mission_avoidance() [next call]
    PLAN->>NAV: OA_SUCCESS + new path
    NAV->>NAV: Update destination to OA path
```

**Update Rates**:
- Main navigation loop: Typically 50-400Hz (vehicle dependent)
- Path planner update: 1Hz (1000ms intervals)
- Result timeout: 3000ms

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:196-254, 257-426`

### Return States

The path planner returns one of four states via `OA_RetState` enum:

| State | Value | Meaning | Vehicle Action |
|-------|-------|---------|----------------|
| `OA_NOT_REQUIRED` | 0 | No obstacles detected, direct path is clear | Continue normal navigation |
| `OA_PROCESSING` | 1 | Computing alternative path | Stop vehicle, wait for result |
| `OA_ERROR` | 2 | Path planning failed or timed out | Stop vehicle, log error |
| `OA_SUCCESS` | 3 | Alternative path found | Navigate to OA destination |

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.h:39-44`

## Path Planning Algorithms

ArduRover supports three path planning configurations selectable via the `OA_TYPE` parameter:

1. **BendyRuler Only** (`OA_TYPE=1`): Fast, reactive avoidance for dynamic obstacles
2. **Dijkstra Only** (`OA_TYPE=2`): Optimal paths around static fence boundaries
3. **Combined Mode** (`OA_TYPE=3`): BendyRuler for proximity, Dijkstra for global planning

### BendyRuler Algorithm

BendyRuler provides real-time reactive obstacle avoidance by searching for safe bearings around detected obstacles.

#### Algorithm Overview

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.h:15`

The BendyRuler algorithm works by:

1. **Lookahead Projection**: Projects vehicle path forward by `OA_BR_LOOKAHEAD` meters
2. **Margin Calculation**: Calculates minimum clearance from obstacles along projected path
3. **Bearing Search**: Tests alternative bearings in both directions to find safe paths
4. **Resistance Logic**: Resists frequent bearing changes to maintain smooth navigation
5. **Best Path Selection**: Selects bearing with maximum margin above threshold

#### Search Modes

**Horizontal Search (XY Plane)**:
- Primary mode for ground vehicles
- Searches bearings left and right of desired heading
- Evaluates clearance from proximity sensors and fence boundaries
- Suitable for navigating around obstacles at vehicle height

**Vertical Search**:
- Searches altitude adjustments up and down
- Used for vehicles capable of elevation changes
- Generally disabled for traditional ground rovers
- May be useful for climbing rovers or underwater vehicles

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.h:24-28, 43-46`

#### Two-Stage Lookahead

BendyRuler uses a two-stage lookahead strategy:

```
Current Position --> Lookahead Step 1 --> Lookahead Step 2 --> Final Destination
                     (shorter, reactive)   (longer, planning)
```

**Step 1**: Immediate obstacle detection (typically `OA_BR_LOOKAHEAD` * 0.5)
**Step 2**: Extended planning horizon (full `OA_BR_LOOKAHEAD` distance)

This approach provides both reactive responses to immediate threats and smoother planning for distant obstacles.

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.cpp` (implementation)

#### Margin Calculation

The algorithm calculates clearance margins from multiple sources:

```cpp
/**
 * @brief Calculate minimum distance between path and obstacles
 * 
 * @details Checks distance from line segment to:
 *          - Proximity sensor detected obstacles
 *          - Circular fence boundaries
 *          - Polygon inclusion/exclusion zones
 *          - Object database entries
 * 
 * @param[in] start Path start location
 * @param[in] end Path end location
 * @param[in] proximity_only If true, only check proximity sensors
 * 
 * @return Minimum clearance in meters, or very large value if clear
 * 
 * @note Returns smallest margin across all obstacle types
 */
float calc_avoidance_margin(const Location &start, const Location &end, 
                           bool proximity_only) const;
```

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.h:49`

**Margin Sources**:
1. **Proximity Sensors**: Real-time ranging data from lidars, sonars, radar
2. **Circular Fence**: Distance to home-centered circular boundary
3. **Polygon Fences**: Distance to inclusion/exclusion polygon edges
4. **Object Database**: Distance to tracked moving objects

#### Bearing Resistance

To prevent oscillating behavior, BendyRuler includes bearing resistance logic:

**Resistance Triggers**:
- Margin improvement ratio < `OA_BR_RATIO` (default 1.5)
- Bearing change > `OA_BR_ANGLE` (default 30 degrees)
- Previous destination still valid

**Behavior**: When resistance is active, the algorithm continues on the previous bearing rather than switching to a marginally better alternative. This creates more stable, predictable vehicle behavior.

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.h:52`

#### Proximity-Only Mode

In combined mode (`OA_TYPE=3`), BendyRuler can operate in proximity-only mode:

```cpp
bool proximity_only = true;  // Initially only check proximity sensors

if (proximity_sensor_detects_obstacle) {
    proximity_only = false;  // Obstacle detected, check all sources
    // Use BendyRuler for immediate avoidance
} else {
    proximity_only = true;   // All clear, fall back to Dijkstra
    // Use Dijkstra for global fence planning
}
```

This mode focuses BendyRuler on dynamic obstacles while allowing Dijkstra to handle static fence planning.

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:357-378`

#### BendyRuler Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `OA_BR_LOOKAHEAD` | 15m | 1-100m | Distance ahead to check for obstacles |
| `OA_BR_RATIO` | 1.5 | 1.0-10.0 | Margin improvement ratio to accept bearing change |
| `OA_BR_ANGLE` | 30° | 0-180° | Maximum angle change before resisting |
| `OA_BR_TYPE` | 1 | 0-2 | Type: 0=Disabled, 1=Horizontal, 2=Vertical |

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.cpp` (var_info)

### Dijkstra's Algorithm

Dijkstra's algorithm provides optimal path planning around static fence boundaries using visibility graphs.

#### Algorithm Overview

**Source**: `/libraries/AC_Avoidance/AP_OADijkstra.h:17`

Dijkstra implementation follows these steps:

1. **Fence Processing**: Load inclusion/exclusion polygons and circles from fence system
2. **Margin Application**: Create offset polygons inside/outside fences with `OA_MARGIN_MAX` clearance
3. **Visibility Graph**: Build graph of line-of-sight connections between fence vertices
4. **Shortest Path**: Run Dijkstra's algorithm to find minimum-distance path from current position to destination
5. **Path Simplification**: Return next waypoint along optimal path

#### Fence Types Supported

**Inclusion Polygons**:
- Define areas where vehicle must remain
- Inner offset polygon created with margin
- Vehicle must stay inside the offset boundary

**Exclusion Polygons**:
- Define no-go zones for the vehicle
- Outer offset polygon created with margin
- Vehicle must stay outside the offset boundary

**Exclusion Circles**:
- Circular no-go zones
- Converted to polygon approximations with margin
- Vehicle must stay outside the circle plus margin

**Source**: `/libraries/AC_Avoidance/AP_OADijkstra.h:73-106`

#### Visibility Graph Construction

The visibility graph connects all fence vertices that have line-of-sight:

```mermaid
graph LR
    A((Current<br/>Position)) 
    B((Vertex 1))
    C((Vertex 2))
    D((Vertex 3))
    E((Destination))
    
    A -.->|blocked| C
    A -->|visible| B
    A -->|visible| D
    B -->|visible| C
    C -->|visible| E
    D -->|visible| E
    D -.->|blocked| B
    
    style A fill:#9f9
    style E fill:#f99
```

**Edge Validation**: Each potential edge is checked to ensure it doesn't intersect any fence boundary.

**Source**: `/libraries/AC_Avoidance/AP_OAVisGraph.cpp` (implementation)

#### Path Calculation

Once the visibility graph is constructed, Dijkstra's shortest path algorithm finds the minimum-distance route:

```
1. Initialize: Set distance[current_position] = 0, all others = infinity
2. Create priority queue with all vertices
3. While destination not reached:
   a. Extract vertex with minimum distance
   b. For each neighbor:
      - Calculate distance via current vertex
      - If shorter than known distance, update and re-queue
4. Backtrack from destination to build final path
5. Return next waypoint along path
```

**Time Complexity**: O((V + E) log V) where V is vertices, E is edges

**Source**: `/libraries/AC_Avoidance/AP_OADijkstra.cpp` (update method)

#### Dynamic Recalculation

Dijkstra path is recalculated when:

1. **Destination Changes**: New waypoint or mission command
2. **Fence Updates**: Polygon or circle fences modified
3. **Manual Trigger**: `recalculate_path()` called by combined mode
4. **Path Deviation**: Vehicle deviates significantly from planned path

The `_shortest_path_ok` flag tracks whether the current path is valid.

**Source**: `/libraries/AC_Avoidance/AP_OADijkstra.h:28`

#### Fast Waypoints Option

When `OA_OPTIONS` bit 2 is set (value 4), Dijkstra provides all intermediate waypoints at once rather than single-step:

**Normal Mode**: Returns only next waypoint, recalculates at each step
**Fast Mode**: Returns full path, allows mission to advance through waypoints quickly

> **Note**: Fast waypoint mode is primarily designed for copters but can be used by rovers for smoother, faster path following.

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.h:77-82`

#### Error Conditions

Dijkstra can encounter several error conditions:

| Error | Cause | Vehicle Response |
|-------|-------|------------------|
| `DIJKSTRA_ERROR_OUT_OF_MEMORY` | Too many fence vertices | Stop, log error |
| `DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS` | Invalid fence geometry | Stop, log error |
| `DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES` | Self-intersecting polygon | Stop, log error |
| `DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS` | Exceeds vertex limit | Stop, log error |
| `DIJKSTRA_ERROR_COULD_NOT_FIND_PATH` | No valid path exists | Stop, log error |

**Source**: `/libraries/AC_Avoidance/AP_OADijkstra.h:54-64`

#### Dijkstra Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `OA_MARGIN_MAX` | 5m | 0.1-100m | Safety margin from fence boundaries |
| `OA_OPTIONS` bit 1 | 0 | 0-1 | Log Dijkstra points for debugging |
| `OA_OPTIONS` bit 2 | 0 | 0-1 | Fast waypoints mode |

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:37-74`

### Combined Mode

Combined mode (`OA_TYPE=3`) leverages the strengths of both algorithms:

#### Operating Logic

```mermaid
stateDiagram-v2
    [*] --> Dijkstra: Normal Operation
    Dijkstra --> BendyRuler: Proximity Sensor<br/>Detects Obstacle
    BendyRuler --> BendyRuler: Obstacle Still Present
    BendyRuler --> RecalculatePath: Obstacle Cleared
    RecalculatePath --> Dijkstra: New Path Computed
    Dijkstra --> [*]: Destination Reached
```

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:357-402`

#### State Transitions

**State 1: Dijkstra Active** (proximity_only = true)
- BendyRuler checks only proximity sensors
- Dijkstra handles global fence-based planning
- Vehicle follows Dijkstra-planned path

**Transition: Proximity Obstacle Detected**
- BendyRuler returns `OA_SUCCESS` with proximity obstacle
- `proximity_only` flag set to `false`
- Switch to BendyRuler state

**State 2: BendyRuler Active** (proximity_only = false)
- BendyRuler checks all obstacle sources
- Vehicle follows BendyRuler immediate avoidance
- Dijkstra planning paused

**Transition: Obstacle Cleared**
- BendyRuler returns failure (no obstacles)
- Call `_oadijkstra->recalculate_path()`
- `proximity_only` flag set to `true`
- Switch back to Dijkstra state

#### Advantages of Combined Mode

1. **Dynamic Obstacle Response**: Immediate reaction to moving or unexpected obstacles
2. **Optimal Static Planning**: Efficient paths around known fence boundaries
3. **Smooth Recovery**: Automatic return to planned path when obstacles clear
4. **Resource Efficiency**: Dijkstra only recalculates when needed

#### Recommended Use Cases

**Combined Mode** is recommended for:
- Missions with both fence boundaries and dynamic obstacles
- Environments with pedestrians, animals, or other moving objects
- Operations requiring optimal path efficiency with safety margin

**BendyRuler Only** is recommended for:
- Simple environments without complex fences
- Situations requiring fastest possible reaction time
- When Dijkstra computational cost is unacceptable

**Dijkstra Only** is recommended for:
- Environments with only static fence boundaries
- Missions requiring provably optimal paths
- When no proximity sensors are available

**Source**: Analysis of `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:311-402`

## Sensor Integration

The obstacle avoidance system integrates multiple sensor types through the proximity sensor framework and object database.

### Proximity Sensors

**Supported Sensor Types**:
- **LightWare Lidar**: I2C and serial SF20, SF40C, SF45 models
- **Benewake Lidar**: TFmini, TF02, TF03 serial lidars
- **Leddar One**: I2C and UART laser rangefinder
- **MaxSonar**: Analog and I2C ultrasonic sensors
- **Teraranger**: I2C and UART laser rangefinders
- **RPLidar**: 360-degree scanning lidar
- **BlueRobotics Ping**: Underwater sonar
- **Ainstein Radar**: FMCW radar sensors
- **SITL Simulated**: For testing in simulator

**Source**: `/libraries/AP_Proximity/` (various driver implementations)

### Sensor Configuration

Proximity sensors are configured via the `PRXn_` parameter group:

```
PRX1_TYPE = 3       # SF40C 360-degree lidar
PRX1_ORIENT = 0     # Forward facing
PRX1_YAW_CORR = 0   # Yaw correction in degrees
PRX1_IGN_ANG1 = 0   # Ignore angle start
PRX1_IGN_WID1 = 0   # Ignore angle width
```

**Key Parameters**:
- `PRXn_TYPE`: Sensor type selection
- `PRXn_ORIENT`: Sensor mounting orientation
- `PRXn_MIN_DIST`: Minimum reliable distance (cm)
- `PRXn_MAX_DIST`: Maximum detection range (cm)
- `PRXn_YAW_CORR`: Yaw angle correction
- `PRXn_IGN_ANGx`: Ignore zones to exclude vehicle structure

### Object Database

The object database (`AP_OADatabase`) maintains a list of detected obstacles with position and velocity information.

#### Database Management

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.cpp`

**Features**:
- Stores up to configured maximum objects
- Tracks position, velocity, and expiration time
- Processes updates from multiple sources
- Provides spatial queries for path planning

**Database Parameters**:
```
OA_DB_SIZE = 10          # Maximum tracked objects
OA_DB_EXPIRE = 10        # Object expiration time (seconds)
OA_DB_QUEUE_SIZE = 50    # Update queue size
```

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.cpp` (var_info)

#### Adding Objects

Objects can be added via MAVLink messages or internal detection:

```cpp
/**
 * @brief Queue an object for addition to database
 * 
 * @param[in] timestamp_ms System time of detection
 * @param[in] object Location and velocity of object
 * 
 * @note Queued objects are processed in background thread
 * @thread_safety Thread-safe via queue mechanism
 */
void queue_push(const Location &loc, const Vector3f &velocity);
```

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.h`

#### Object Expiration

Objects automatically expire after `OA_DB_EXPIRE` seconds without updates. This prevents stale data from affecting path planning.

### Sensor Fusion

The avoidance algorithms fuse data from multiple sources:

```mermaid
graph TD
    PROX1[Proximity Sensor 1] --> PROXLIB[AP_Proximity]
    PROX2[Proximity Sensor 2] --> PROXLIB
    MAVLINK[MAVLink Objects] --> OADB[Object Database]
    PROXLIB --> OADB
    
    OADB --> BENDY[BendyRuler]
    FENCE[Fence System] --> BENDY
    FENCE --> DIJK[Dijkstra]
    
    BENDY --> MARGIN[Margin Calculation]
    DIJK --> PATH[Path Selection]
    
    MARGIN --> DECISION{Sufficient<br/>Clearance?}
    PATH --> DECISION
    
    DECISION -->|Yes| SAFE[Safe Path]
    DECISION -->|No| AVOID[Avoidance Maneuver]
```

### Coordinate Systems

All obstacle positions are stored in NED (North-East-Down) coordinates relative to the EKF origin:

**Conversions**:
- Proximity sensors provide range and bearing
- Converted to NED position using vehicle attitude
- Stored as `Location` with latitude, longitude, altitude
- Path planning operates in local NED coordinates

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.cpp` (coordinate transformations)

## Integration with Navigation

### AR_WPNav_OA Integration

The `AR_WPNav_OA` class extends `AR_WPNav` to add obstacle avoidance capabilities.

#### Update Cycle

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:26-150`

```cpp
/**
 * @brief Update navigation with obstacle avoidance
 * 
 * @details Called at main navigation rate (typically 50Hz)
 *          - Calls path planner for alternative destination
 *          - Switches to OA destination if obstacle detected
 *          - Restores original destination when clear
 *          - Stops vehicle if path planning fails
 * 
 * @param[in] dt Time since last update (seconds)
 * 
 * @note Must be armed and have valid destination
 * @warning Vehicle will stop if OA returns OA_ERROR
 */
void AR_WPNav_OA::update(float dt) override;
```

#### Destination Management

The navigation system maintains three sets of destinations:

**Normal Operation**:
- `_origin`: Path segment start
- `_destination`: Target waypoint
- `_next_destination`: Following waypoint (for cornering)

**OA Active**:
- `_origin_oabak`: Backup of original origin
- `_destination_oabak`: Backup of original destination
- `_next_destination_oabak`: Backup of next destination
- `_oa_origin`: OA-adjusted origin
- `_oa_destination`: OA-adjusted destination (vehicle navigates here)
- `_oa_next_destination`: OA-adjusted next destination

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.h:36-44`

#### State Machine

```mermaid
stateDiagram-v2
    [*] --> Inactive: set_desired_location()
    Inactive --> Processing: mission_avoidance()<br/>returns OA_PROCESSING
    Processing --> Processing: Computing path
    Processing --> Error: Timeout (3 seconds)
    Processing --> Active: OA_SUCCESS returned
    Active --> Active: Following OA path
    Active --> Inactive: OA_NOT_REQUIRED<br/>(obstacle cleared)
    Error --> Inactive: set_desired_location()
    
    note right of Inactive
        _oa_active = false
        Following normal waypoints
    end note
    
    note right of Active
        _oa_active = true
        Following OA destination
    end note
    
    note right of Processing
        Vehicle stopped
        Waiting for path
    end note
    
    note right of Error
        Vehicle stopped
        Error logged
    end note
```

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:65-134`

### Mission Integration

Obstacle avoidance works transparently with the mission system:

**Mission Commands Supported**:
- `MAV_CMD_NAV_WAYPOINT`: Navigate to waypoint with OA
- `MAV_CMD_NAV_RETURN_TO_LAUNCH`: RTL with OA
- `MAV_CMD_NAV_LOITER_UNLIM`: Loiter with OA active
- `MAV_CMD_DO_SET_HOME`: Set home location

**Mission Behavior**:
1. Mission provides destination and next_destination
2. OA system calculates intermediate waypoints
3. Vehicle navigates through OA waypoints
4. When path clears, vehicle resumes direct path
5. Mission advances when final destination reached

**Important**: The mission does NOT advance while following OA intermediate destinations. Only when `reached_destination()` returns true (OA inactive and at final waypoint) does the mission progress.

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:167-175`

### Mode Compatibility

**Fully Compatible Modes**:
- **Auto**: Full OA support with mission waypoints
- **Guided**: OA works with guided mode destinations
- **RTL**: Return to launch uses OA to avoid obstacles
- **SmartRTL**: Smart return to launch with OA

**Partially Compatible Modes**:
- **Steering**: Manual steering, OA not applied
- **Acro**: Manual control, OA not applied
- **Hold**: Stationary, OA not needed

**Source**: Rover mode implementations in `/Rover/mode_*.cpp`

### WP_RESET Option

The `OA_OPTIONS` bit 0 provides waypoint reset functionality for rovers:

```
OA_OPTIONS = 1    # Enable WP_RESET
```

**Behavior**: When set, after obstacle avoidance ends, the origin of the current waypoint segment is reset to the vehicle's current position rather than the original origin. This prevents the vehicle from attempting to return to its original path, which may still contain obstacles.

**Use Case**: Environments with dense obstacles where backtracking is undesirable.

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.h:78`, `/libraries/AR_WPNav/AR_WPNav_OA.cpp:76`

## Configuration Parameters

### Core Parameters

**OA_TYPE** - Path Planning Algorithm Selection
```
Range: 0-3
Default: 0 (Disabled)
Values:
  0 = Disabled (no obstacle avoidance)
  1 = BendyRuler only
  2 = Dijkstra only
  3 = Dijkstra with BendyRuler
```
**Description**: Selects which path planning algorithm(s) to use. Combined mode (3) is recommended for most applications.

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:39-44`

---

**OA_MARGIN_MAX** - Maximum Obstacle Detection Range
```
Range: 0.1-100 meters
Default: 5 meters
Units: meters
```
**Description**: Obstacles beyond this distance are ignored. Also used as safety margin for Dijkstra fence offsets. Increase for higher speed operation, decrease for tighter spaces.

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:49-56`

---

**OA_OPTIONS** - Recovery and Behavior Options
```
Range: 0-7 (bitmask)
Default: 1
Bitmask:
  Bit 0 (value 1) = Reset waypoint origin after avoidance
  Bit 1 (value 2) = Log Dijkstra points for debugging
  Bit 2 (value 4) = Fast waypoints mode (Dijkstra)
```
**Description**: Configures obstacle avoidance behavior during and after avoidance maneuvers.

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:62-68`

### BendyRuler Parameters

**OA_BR_LOOKAHEAD** - Search Distance
```
Range: 1-100 meters
Default: 15 meters
Units: meters
```
**Description**: How far ahead BendyRuler projects the vehicle path when searching for obstacles. Increase for higher speeds, decrease for tighter maneuvering.

**Recommended Values**:
- Slow vehicles (<2 m/s): 10-15m
- Medium vehicles (2-5 m/s): 15-25m
- Fast vehicles (>5 m/s): 25-40m

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.cpp` (var_info)

---

**OA_BR_RATIO** - Margin Improvement Ratio
```
Range: 1.0-10.0
Default: 1.5
Units: ratio
```
**Description**: BendyRuler will only change bearing if the new margin is at least this ratio better than the previous margin. Higher values create more stable paths but may be slower to react.

**Tuning**:
- Increase (2.0-3.0) if vehicle oscillates or hunts
- Decrease (1.1-1.3) if vehicle reacts too slowly

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.cpp` (var_info)

---

**OA_BR_ANGLE** - Bearing Change Resistance
```
Range: 0-180 degrees
Default: 30 degrees
Units: degrees
```
**Description**: BendyRuler will resist bearing changes larger than this angle unless margin improvement exceeds the ratio threshold. Prevents large, rapid direction changes.

**Tuning**:
- Increase (45-90°) for more stable, gradual turns
- Decrease (10-20°) for more aggressive avoidance

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.cpp` (var_info)

---

**OA_BR_TYPE** - BendyRuler Search Type
```
Range: 0-2
Default: 1 (Horizontal)
Values:
  0 = Disabled
  1 = Horizontal (XY plane search)
  2 = Vertical (altitude search)
```
**Description**: Selects search plane for BendyRuler. Most rovers use horizontal only.

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.h:24-28`

### Object Database Parameters

**OA_DB_SIZE** - Maximum Tracked Objects
```
Range: 1-100
Default: 10
Units: objects
```
**Description**: Maximum number of obstacles stored in the object database. Increase for crowded environments, decrease to save memory.

**Memory Impact**: Each object requires approximately 50 bytes.

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.cpp` (var_info)

---

**OA_DB_EXPIRE** - Object Expiration Time
```
Range: 1-127 seconds
Default: 10 seconds
Units: seconds
```
**Description**: Objects not updated within this time are removed from the database. Shorter times keep database fresh, longer times maintain awareness of slow-moving obstacles.

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.cpp` (var_info)

---

**OA_DB_QUEUE_SIZE** - Update Queue Size
```
Range: 10-200
Default: 50
Units: queue entries
```
**Description**: Size of queue for object updates from sensors. Increase if objects are being dropped in high-density environments.

**Source**: `/libraries/AC_Avoidance/AP_OADatabase.cpp` (var_info)

### Proximity Sensor Parameters

**PRXn_TYPE** - Sensor Type
```
Range: 0-14
Default: 0 (None)
```
**Description**: Selects proximity sensor type. See AP_Proximity library documentation for supported sensors.

---

**PRXn_ORIENT** - Sensor Orientation
```
Range: 0-39
Default: 0 (Forward)
```
**Description**: Mounting orientation of the sensor relative to vehicle body frame.

---

**PRXn_YAW_CORR** - Yaw Correction
```
Range: -180 to +180 degrees
Default: 0
Units: degrees
```
**Description**: Yaw angle correction for sensor alignment errors.

---

**PRXn_IGN_ANG1/2/3** - Ignore Angle Start
```
Range: 0-360 degrees
Default: 0
Units: degrees
```
**Description**: Start angle of region to ignore (e.g., vehicle body). Up to 3 ignore zones supported.

---

**PRXn_IGN_WID1/2/3** - Ignore Width
```
Range: 0-127 degrees
Default: 0
Units: degrees
```
**Description**: Width of ignore zone. Objects detected in ignore zones are not added to database.

**Source**: `/libraries/AP_Proximity/AP_Proximity_Params.cpp`

## Tuning and Optimization

### Initial Configuration

**Step 1: Enable Obstacle Avoidance**
```
OA_TYPE = 1          # Start with BendyRuler only
OA_MARGIN_MAX = 5    # 5 meter detection range
OA_BR_LOOKAHEAD = 15 # 15 meter lookahead
```

**Step 2: Configure Proximity Sensors**
```
PRX1_TYPE = 3        # SF40C or your sensor type
PRX1_ORIENT = 0      # Forward facing
PRX1_MIN_DIST = 50   # 50cm minimum
PRX1_MAX_DIST = 1500 # 15m maximum
```

**Step 3: Test in Safe Environment**
- Place obstacles at known distances
- Drive toward obstacle in manual mode
- Verify sensor readings in ground station
- Switch to Auto mode with simple mission
- Observe avoidance behavior

### BendyRuler Tuning

#### Symptom: Vehicle oscillates around obstacles

**Cause**: Margin ratio too low or angle threshold too low

**Solution**:
```
OA_BR_RATIO = 2.0    # Increase from default 1.5
OA_BR_ANGLE = 45     # Increase from default 30
```

**Expected Result**: Smoother, less oscillatory paths with fewer bearing changes

---

#### Symptom: Vehicle doesn't react fast enough

**Cause**: Lookahead too long or ratio too high

**Solution**:
```
OA_BR_LOOKAHEAD = 10  # Decrease for faster reaction
OA_BR_RATIO = 1.2     # Allow smaller margin improvements
```

**Expected Result**: Earlier obstacle detection and more aggressive avoidance

---

#### Symptom: Vehicle stops frequently with no visible obstacles

**Cause**: Sensor noise, margin too large, or spurious detections

**Solution**:
```
OA_MARGIN_MAX = 3     # Reduce detection range
PRX1_MIN_DIST = 100   # Increase minimum distance to filter noise
```

Add ignore zones for vehicle structure:
```
PRX1_IGN_ANG1 = 135   # Start of ignore zone
PRX1_IGN_WID1 = 90    # 90 degree width (rear of vehicle)
```

**Expected Result**: Fewer false positives, smoother operation

---

#### Symptom: Vehicle takes very wide paths around obstacles

**Cause**: Margin max too large, lookahead too long

**Solution**:
```
OA_MARGIN_MAX = 2     # Tighter clearance
OA_BR_LOOKAHEAD = 12  # Shorter planning horizon
```

**Expected Result**: More efficient, tighter paths around obstacles

### Dijkstra Tuning

#### Symptom: Vehicle stops with "could not find path" error

**Cause**: Fences configured with no valid path, or margin too large

**Solution 1**: Reduce margin
```
OA_MARGIN_MAX = 2     # Reduce from default 5m
```

**Solution 2**: Check fence configuration
- Verify inclusion fence is not too small
- Verify exclusion fences don't block all paths
- Check for overlapping or self-intersecting polygons

**Expected Result**: Path found through available space

---

#### Symptom: Vehicle paths are inefficient or erratic

**Cause**: Too many fence vertices or complex geometry

**Solution**:
- Simplify fence polygons (fewer vertices)
- Use circular fences where appropriate
- Enable debug logging to visualize path:
```
OA_OPTIONS = 2        # Enable Dijkstra point logging
```

**Expected Result**: More efficient, logical paths

---

#### Symptom: Dijkstra computation taking too long

**Cause**: Too many fence vertices or complex visibility graph

**Solution**:
- Reduce number of fence points
- Simplify fence geometry
- Consider switching to BendyRuler only for simpler environments

**Expected Result**: Faster path computation, less latency

### Combined Mode Tuning

**Recommended Settings for Combined Mode**:
```
OA_TYPE = 3                # Dijkstra with BendyRuler
OA_MARGIN_MAX = 4          # Moderate margin
OA_BR_LOOKAHEAD = 15       # Standard lookahead
OA_BR_RATIO = 1.5          # Balanced resistance
OA_BR_ANGLE = 30           # Moderate angle threshold
OA_OPTIONS = 1             # WP reset enabled
```

**Tuning Strategy**:
1. Start with BendyRuler parameters tuned for your environment
2. Add Dijkstra with simple fence geometry
3. Test fence-only scenarios (verify Dijkstra paths)
4. Test proximity-only scenarios (verify BendyRuler)
5. Test combined scenarios (verify handoff)

**Handoff Verification**:
- Drive toward fence with proximity obstacle in path
- BendyRuler should activate first (proximity detection)
- After avoiding proximity obstacle, Dijkstra should resume
- Vehicle should follow Dijkstra path to destination

### Performance Optimization

#### For Maximum Safety (Conservative)
```
OA_TYPE = 3              # Combined mode
OA_MARGIN_MAX = 8        # Large safety margin
OA_BR_LOOKAHEAD = 25     # Long lookahead
OA_BR_RATIO = 2.5        # High resistance to changes
OA_BR_ANGLE = 60         # Large angle resistance
```
**Use Case**: High-value vehicles, populated areas, high speeds

---

#### For Tight Spaces (Aggressive)
```
OA_TYPE = 1              # BendyRuler only
OA_MARGIN_MAX = 1.5      # Tight clearance
OA_BR_LOOKAHEAD = 8      # Short lookahead
OA_BR_RATIO = 1.2        # Quick bearing changes
OA_BR_ANGLE = 15         # Small angle resistance
```
**Use Case**: Warehouse robots, indoor navigation, slow speeds

---

#### For Long-Range Missions (Efficient)
```
OA_TYPE = 2              # Dijkstra only
OA_MARGIN_MAX = 5        # Moderate margin
OA_OPTIONS = 5           # WP reset + fast waypoints
```
**Use Case**: Outdoor missions with static fences, minimal dynamic obstacles

---

#### For Dynamic Environments (Responsive)
```
OA_TYPE = 3              # Combined mode
OA_MARGIN_MAX = 4        # Moderate margin
OA_BR_LOOKAHEAD = 12     # Quick response
OA_BR_RATIO = 1.3        # Low resistance
OA_BR_ANGLE = 20         # Small angle threshold
OA_DB_EXPIRE = 5         # Quick expiration for dynamic objects
```
**Use Case**: Pedestrian areas, environments with moving obstacles

## Operational Modes

### Normal Navigation Without Obstacles

When no obstacles are detected and OA is enabled but not required:

**Return State**: `OA_NOT_REQUIRED`

**Vehicle Behavior**:
- `_oa_active` remains false
- Navigation follows standard `AR_WPNav` behavior
- Path planner continues monitoring in background
- Zero performance impact on main control loop

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:67-78`

### Active Avoidance

When obstacles are detected and avoidance is active:

**Return State**: `OA_SUCCESS`

**Vehicle Behavior**:
- `_oa_active` set to true
- `set_desired_location()` called with OA-adjusted destination
- Vehicle navigates toward intermediate OA waypoint
- Original destination stored in backup variables
- Bearing and distance calculated to OA destination

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:87-131`

### Processing State

When path planner is computing a new path:

**Return State**: `OA_PROCESSING`

**Vehicle Behavior**:
- Vehicle decelerates to stop
- Turn rate set to zero
- Waits for path planner to return result
- Timeout after 3 seconds triggers error state

**Purpose**: Prevents vehicle from continuing toward obstacle while path is computed

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:80-85, 139-145`

### Error State

When path planning fails or times out:

**Return State**: `OA_ERROR`

**Vehicle Behavior**:
- Vehicle stops immediately
- Error logged to ground station and dataflash
- Requires manual intervention or destination change
- Common causes:
  - Path planner timeout (>3 seconds)
  - No valid path exists
  - Dijkstra error (geometry, memory, etc.)

**Recovery**: Set new destination or switch to manual mode

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:80-85, 139-145`

### Recovery to Normal Navigation

When obstacles clear during active avoidance:

**Transition**: `OA_SUCCESS` → `OA_NOT_REQUIRED`

**Vehicle Behavior**:
- Restore original destination from backup
- Call `set_desired_location(_destination_oabak)`
- `_oa_active` set to false
- Resume normal navigation toward original waypoint

**Optional WP Reset**: If `OA_OPTIONS` bit 0 is set, origin resets to current position rather than restoring original origin

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:68-77`

## Error Handling and Recovery

### Path Planner Errors

**Error Detection**: Path planner monitors for multiple error conditions:

1. **Initialization Failures**
   - Algorithm failed to allocate memory
   - Pre-arm check detects uninitialized algorithms
   - **Recovery**: Reboot vehicle to retry initialization

2. **Timeout Errors**
   - Path computation exceeds 3 seconds
   - Background thread not responding
   - **Recovery**: Vehicle stops, waits for next cycle or manual intervention

3. **Geometry Errors** (Dijkstra-specific)
   - Overlapping polygon points
   - Self-intersecting polygon lines
   - Too many fence vertices (exceeds limit)
   - **Recovery**: Simplify fence geometry or disable Dijkstra

4. **No Valid Path**
   - All paths blocked by obstacles/fences
   - Destination unreachable
   - **Recovery**: Reduce `OA_MARGIN_MAX` or move obstacles

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:124-150`, `/libraries/AC_Avoidance/AP_OADijkstra.h:54-70`

### Pre-Arm Checks

Before arming, obstacle avoidance system performs validation:

```cpp
/**
 * @brief Pre-arm check for obstacle avoidance initialization
 * 
 * @details Verifies:
 *          - Configured algorithms successfully initialized
 *          - BendyRuler allocated if TYPE is 1 or 3
 *          - Dijkstra allocated if TYPE is 2 or 3
 *          - Background thread created
 * 
 * @param[out] failure_msg Buffer for error message
 * @param[in]  failure_msg_len Length of message buffer
 * 
 * @return true if pre-arm check passed, false otherwise
 * 
 * @warning If check fails, reboot required after fixing configuration
 */
bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;
```

**Failure Messages**:
- "BendyRuler OA requires reboot" - Algorithm allocation failed
- "Dijkstra OA requires reboot" - Algorithm allocation failed
- "OA requires reboot" - Combined mode initialization failed

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:124-151`

### Vehicle Stop Conditions

The vehicle will stop (decelerate to zero) under these conditions:

1. **OA_PROCESSING State**
   - Path planner computing new path
   - Vehicle waits for result before proceeding
   - **Duration**: Typically 1-2 seconds, max 3 seconds

2. **OA_ERROR State**
   - Path planning failed
   - Timeout occurred
   - No valid path found
   - **Duration**: Until destination changed or manual takeover

3. **Internal Errors**
   - Flow of control errors (should never happen)
   - `set_desired_location()` failures during OA
   - **Action**: Logged as internal error, vehicle stops

**Source**: `/libraries/AR_WPNav/AR_WPNav_OA.cpp:73, 83, 96, 110, 126, 139-145`

### Logging and Diagnostics

**DataFlash Logging**:

The system logs comprehensive information to dataflash:

**OABendyRuler Messages** (when logging enabled):
- Type (horizontal/vertical)
- Active status
- Target yaw and pitch
- Resistance flag
- Calculated margin
- Final destination
- OA destination

**OADijkstra Messages** (when `OA_OPTIONS` bit 1 set):
- Visibility graph nodes
- Computed path points
- Fence vertices
- Error conditions

**Source**: `/libraries/AC_Avoidance/AP_OABendyRuler.h:75-78`

**GCS Messages**:

Ground station receives status via MAVLink:
- `OBSTACLE_DISTANCE` messages with proximity sensor data
- `DEBUG_VECT` messages with path planning vectors (when enabled)
- Text messages for errors and state changes

### Recovery Procedures

**Manual Recovery**:
1. Switch to HOLD or MANUAL mode
2. Manually drive clear of obstacles
3. Switch back to AUTO to resume mission

**Parameter Adjustment Recovery**:
1. Reduce `OA_MARGIN_MAX` to allow tighter clearance
2. Adjust sensor ignore zones if false positives detected
3. Simplify fence geometry if Dijkstra fails

**Mission Replanning**:
1. Upload new mission with waypoints avoiding problem areas
2. Resume mission from current location
3. System will plan paths for new waypoints

**Emergency Recovery**:
1. Disable obstacle avoidance: `OA_TYPE = 0`
2. Complete mission without OA
3. Debug and reconfigure OA for next mission

## Performance Considerations

### Computational Cost

**BendyRuler Algorithm**:
- **CPU Usage**: Low to moderate (typically <5ms per update)
- **Update Rate**: 1Hz (1000ms intervals)
- **Memory**: ~2KB static allocation
- **Scalability**: Linear with number of test bearings and proximity sensor rays

**Dijkstra Algorithm**:
- **CPU Usage**: Moderate to high (5-50ms depending on fence complexity)
- **Update Rate**: 1Hz, recalculated only when needed
- **Memory**: Proportional to fence vertices (100 bytes per vertex typical)
- **Scalability**: O((V + E) log V) where V=vertices, E=edges

**Combined Mode**:
- **CPU Usage**: Sum of both algorithms when proximity triggers BendyRuler
- **Typical**: Dijkstra overhead unless proximity obstacle detected
- **Peak**: Both algorithms running simultaneously during transition

**Source**: Analysis of algorithm implementations

### Threading Model

**Main Thread**:
- Navigation control loop (50-400Hz)
- Calls `mission_avoidance()` each cycle (lightweight check)
- Updates vehicle control based on OA destination
- Minimal latency (<1ms typical)

**Avoidance Thread**:
- Background thread priority: `PRIORITY_IO`
- Stack size: 8192 bytes
- Update rate: 1Hz (1000ms intervals)
- Runs algorithms without blocking main thread
- Thread-safe via semaphore protection

**Semaphore Protection**:
```cpp
HAL_Semaphore _rsem;  // Protects avoidance_request and avoidance_result

// Main thread locks briefly to:
//   - Copy request data
//   - Read result data

// Avoidance thread locks briefly to:
//   - Read request data
//   - Write result data
```

**Lock Duration**: Microseconds (copying small structures)

**Source**: `/libraries/AC_Avoidance/AP_OAPathPlanner.cpp:153-174, 257-426`

### Memory Usage

**Static Allocations**:
- `AP_OAPathPlanner`: ~500 bytes
- `AP_OABendyRuler`: ~200 bytes (if enabled)
- `AP_OADijkstra`: ~1000 bytes base (if enabled)
- `AP_OADatabase`: ~50 bytes per object × `OA_DB_SIZE`
- Thread stack: 8192 bytes

**Dynamic Allocations**:
- Dijkstra visibility graph: ~100 bytes per fence vertex
- Object database entries: ~50 bytes × `OA_DB_SIZE` (default 500 bytes)
- Update queue: ~20 bytes × `OA_DB_QUEUE_SIZE` (default 1000 bytes)

**Total Memory Estimate**:
- BendyRuler only: ~3KB
- Dijkstra only: ~5KB + (100 × fence vertices)
- Combined mode: ~6KB + (100 × fence vertices)
- Object database: +1.5KB (default settings)

**Memory Constraints**:
- Low-memory boards: Use BendyRuler only, reduce `OA_DB_SIZE`
- Standard boards: Combined mode with moderate fence complexity
- High-memory boards: Complex fences, large object database

**Source**: Code structure analysis of AC_Avoidance libraries

### Real-Time Performance

**Latency Budget**:
1. Sensor reading → Object database: <50ms
2. Object database → Path planner: 0ms (continuous)
3. Path planner computation: <1000ms (1Hz update)
4. Path planner → Navigation: <1ms (semaphore-protected copy)
5. Navigation → Control output: <10ms (main loop rate)

**Total Latency**: ~50-1100ms from sensor detection to control response

**Worst Case Scenarios**:
- Complex Dijkstra with 50+ fence vertices: up to 50ms computation
- BendyRuler searching 180-degree arc: up to 10ms
- Combined mode transition: up to 60ms for recalculation trigger

**Real-Time Considerations**:
- Obstacle avoidance is NOT hard real-time
- Vehicle stops during path computation to maintain safety
- 1Hz update rate sufficient for ground vehicle speeds (<20 m/s)
- Faster updates not beneficial due to sensor latency

### Scalability Limits

**Fence Complexity**:
- Maximum recommended vertices: 100 per polygon
- Maximum recommended polygons: 10
- Beyond these limits, Dijkstra performance degrades significantly

**Proximity Sensors**:
- Maximum sensors: 8 (hardware/software limit)
- Each sensor can detect multiple obstacles
- Object database limits total tracked objects

**Object Database**:
- Default maximum: 10 objects
- Configurable up to 100 objects
- Each object requires update processing
- Higher counts increase memory and CPU usage

**Benchmark Data** (typical performance on STM32F7):
- BendyRuler with 8 proximity rays: 3-5ms
- Dijkstra with 20 fence vertices: 10-15ms
- Dijkstra with 50 fence vertices: 30-40ms
- Object database with 10 objects: 1-2ms update

## Testing and Validation

### Simulation Testing (SITL)

**Setup SITL with Obstacle Avoidance**:

```bash
# Start SITL with proximity sensor simulation
sim_vehicle.py -v Rover --console --map

# In MAVProxy console:
param set OA_TYPE 1          # Enable BendyRuler
param set OA_BR_LOOKAHEAD 15 # 15m lookahead
param set OA_MARGIN_MAX 5    # 5m margin
param set PRX1_TYPE 10       # SITL proximity sensor
param set PRX1_ORIENT 0      # Forward facing
param write                  # Save parameters
```

**Create Obstacles in SITL**:

```python
# Use MAVProxy to create proximity obstacles
fence enable
fence list
fence circle 35.363261 149.165230 50  # 50m radius exclusion
```

**Test Scenarios**:

1. **Single Obstacle Avoidance**
   - Upload waypoint on far side of obstacle
   - Arm and switch to AUTO
   - Verify vehicle detects and avoids obstacle
   - Verify vehicle returns to path after avoidance

2. **Narrow Passage**
   - Create two fences with narrow gap
   - Upload waypoint beyond gap
   - Verify vehicle finds path through gap

3. **No Valid Path**
   - Completely surround destination with fences
   - Verify vehicle stops and reports error

4. **Combined Mode Handoff**
   - Use fences for global planning
   - Add proximity obstacle on planned path
   - Verify switch from Dijkstra to BendyRuler
   - Verify return to Dijkstra after avoidance

**Source**: ArduPilot SITL documentation

### Hardware Testing

**Progressive Testing Approach**:

**Phase 1: Sensor Validation**
- Power on vehicle with OA disabled
- Monitor proximity sensor readings in GCS
- Place obstacles at known distances (1m, 2m, 5m, 10m)
- Verify sensor readings match actual distances
- Verify ignore zones work correctly (vehicle structure not detected)

**Phase 2: Static Obstacle Testing**
- Enable `OA_TYPE = 1` (BendyRuler only)
- Place single obstacle in vehicle path
- Drive toward obstacle in MANUAL mode
- Switch to AUTO mode at 20m distance
- Verify detection, stop, avoidance, and resumption

**Phase 3: Multiple Obstacles**
- Create obstacle course with multiple objects
- Upload mission with waypoints through course
- Run mission in AUTO mode
- Verify vehicle navigates through entire course
- Check dataflash logs for OA activation patterns

**Phase 4: Fence Integration**
- Configure inclusion and exclusion fences
- Enable `OA_TYPE = 2` (Dijkstra only)
- Upload mission with waypoints requiring fence planning
- Verify paths stay within fences with proper margin
- Test with progressively complex fence geometry

**Phase 5: Combined Mode**
- Enable `OA_TYPE = 3`
- Create scenario with both fences and dynamic obstacles
- Verify proper algorithm selection and handoff
- Test recovery behavior after clearing obstacles

**Safety Precautions**:
- Always have manual RC override ready
- Test in open area with clear escape paths
- Start with low speeds (1-2 m/s)
- Gradually increase complexity and speed
- Monitor battery level and failsafes

### Validation Checklist

**Functional Requirements**:
- [ ] Vehicle detects obstacles at configured range
- [ ] Vehicle stops when processing path
- [ ] Vehicle navigates around single obstacle
- [ ] Vehicle handles multiple obstacles
- [ ] Vehicle respects fence boundaries
- [ ] Vehicle maintains safety margins
- [ ] Vehicle returns to planned path after avoidance
- [ ] Mission advances only at final destination
- [ ] Error states stop vehicle safely
- [ ] Pre-arm checks catch initialization failures

**Performance Requirements**:
- [ ] Path computation completes within 3 seconds
- [ ] Sensor updates occur at >1Hz
- [ ] OA overhead <10% CPU usage
- [ ] Memory usage within board limits
- [ ] No stack overflows or memory leaks
- [ ] Smooth transitions between states
- [ ] Minimal oscillation or hunting

**Robustness Requirements**:
- [ ] Handles sensor dropouts gracefully
- [ ] Recovers from transient errors
- [ ] Operates correctly at vehicle speed limits
- [ ] Functions in GPS-denied environments (with non-GPS position source)
- [ ] Handles fence configuration changes
- [ ] Tolerates parameter changes during operation
- [ ] Proper behavior with partial sensor coverage

### Debugging Tools

**Real-Time Monitoring**:
- Mission Planner: "Proximity" display shows sensor coverage
- MAVProxy: `proximity` command displays raw sensor data
- QGroundControl: Obstacle distance visualization

**Log Analysis**:
- Download dataflash logs after test flight
- Analyze `OABendyRuler` messages for BendyRuler behavior
- Analyze `OADijkstra` messages for Dijkstra paths (if logging enabled)
- Cross-reference with `GPS`, `CTUN`, `NTUN` messages

**Parameter Logging**:
Enable detailed logging with:
```
OA_OPTIONS = 2    # Log Dijkstra points
LOG_BITMASK = 131071  # Enable all logging
```

**Common Issues and Solutions**:

| Symptom | Possible Cause | Debug Steps |
|---------|---------------|-------------|
| OA never activates | Sensor not configured | Check `PRX1_TYPE`, verify sensor readings |
| Vehicle stops repeatedly | Spurious detections | Check ignore zones, sensor min distance |
| "Requires reboot" message | Init failure | Check available memory, simplify config |
| Path computation timeouts | Dijkstra too complex | Reduce fence vertices, check for overlaps |
| Oscillating behavior | Tuning too aggressive | Increase `OA_BR_RATIO` and `OA_BR_ANGLE` |
| Wide paths around obstacles | Margin too large | Reduce `OA_MARGIN_MAX` |

---

## Summary

The ArduRover obstacle avoidance system provides comprehensive real-time collision prevention through:

- **Dual Algorithm Approach**: BendyRuler for dynamic obstacles, Dijkstra for static fence planning
- **Multi-Sensor Fusion**: Integrates proximity sensors, fence boundaries, and object database
- **Threaded Architecture**: Background processing doesn't block control loop
- **Seamless Integration**: Works transparently with missions and navigation
- **Configurable Behavior**: Extensive parameters for tuning to specific applications

**Key Takeaways**:
- Start with BendyRuler only for simplest setup
- Use combined mode for best of both worlds
- Tune lookahead and margins for your vehicle speed
- Test progressively from simple to complex scenarios
- Monitor logs to understand and debug behavior

**Further Reading**:
- `/libraries/AC_Avoidance/README.md` - Algorithm details and theory
- `/Rover/docs/navigation.md` - Navigation system architecture
- `/Rover/docs/path_planning.md` - Detailed path planning documentation
- ArduPilot Wiki: Obstacle Avoidance sections

**Source Code References**:
- Core path planning: `/libraries/AC_Avoidance/AP_OAPathPlanner.{cpp,h}`
- BendyRuler: `/libraries/AC_Avoidance/AP_OABendyRuler.{cpp,h}`
- Dijkstra: `/libraries/AC_Avoidance/AP_OADijkstra.{cpp,h}`
- Rover integration: `/libraries/AR_WPNav/AR_WPNav_OA.{cpp,h}`
- Object database: `/libraries/AC_Avoidance/AP_OADatabase.{cpp,h}`
- Proximity sensors: `/libraries/AP_Proximity/`

---

*This documentation is part of the ArduPilot Ground Vehicle Documentation Project*

*For questions or contributions, see CONTRIBUTING.md*
