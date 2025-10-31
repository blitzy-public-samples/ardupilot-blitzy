# ArduPilot ROS 2 packages

This directory contains ROS 2 packages and configuration files for running
ROS 2 processes and nodes that communicate with the ArduPilot DDS client
library using the microROS agent. It contains the following packages:
 
#### `ardupilot_sitl`

This is a `colcon` package for building and running ArduPilot SITL using the ROS 2 CLI.
For example `ardurover` SITL may be launched with:

```bash
ros2 launch ardupilot_sitl sitl.launch.py command:=ardurover model:=rover
```

Other launch files are included with many arguments.
Some common arguments are exposed and forwarded to the underlying process.

For example, MAVProxy can be launched, and you can enable the `console` and `map`.
```bash
ros2 launch ardupilot_sitl sitl_mavproxy.launch.py map:=True console:=True 
```

ArduPilot SITL does not yet expose all arguments from the underlying binary.
See [#27714](https://github.com/ArduPilot/ardupilot/issues/27714) for context.

To see all current options, use the `-s` argument:
```bash
ros2 launch ardupilot_sitl sitl.launch.py -s
```

#### `ardupilot_dds_tests`

A `colcon` package for testing communication between `micro_ros_agent` and the
ArduPilot `AP_DDS` client library.

## Prerequisites

The packages depend on:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## Install Ubuntu

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

The ROS 2 tutorials contain more details regarding [ROS 2 workspaces](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html).

#### 2. Get the `ros2.repos` file

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
vcs import --recursive < ros2.repos
```

#### 3. Update dependencies

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --rosdistro ${ROS_DISTRO} --from-paths src
```

#### 4. Build

Check that the [ROS environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#check-environment-variables) is configured correctly:

```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_TESTING=ON
```

#### 5. Test

```bash
source ./install/setup.bash
colcon test --packages-select ardupilot_dds_tests
colcon test-result --all --verbose
```

To debug a specific test, you can do the following:
```
colcon --log-level DEBUG test --packages-select ardupilot_dds_tests --event-handlers=console_direct+ --pytest-args -k test_dds_udp_geopose_msg_recv -s
```

## Install macOS

The install procedure on macOS is similar, except that all dependencies
must be built from source and additional compiler flags are needed.

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

#### 2. Get the `ros2_macos.repos` file

The `ros2_macos.repos` includes additional dependencies to build:

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2_macos.repos
vcs import --recursive < ros2_macos.repos
```

#### 3. Update dependencies

```bash
cd ~/ros2_ws
source /{path_to_your_ros_distro_workspace}/install/setup.zsh
```

#### 4.1. Build microxrcedds_gen:

```bash
cd ~/ros2_ws/src/microxrcedds_gen
./gradlew assemble
export PATH=$PATH:$(pwd)/scripts
```

#### 4.2. Build colcon projects

```bash
colcon build --symlink-install --cmake-args \
-DBUILD_TESTING=ON \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DCMAKE_MACOSX_RPATH=FALSE \
-DUAGENT_SOCKETCAN_PROFILE=OFF \
-DUAGENT_LOGGER_PROFILE=OFF \
-DUAGENT_USE_SYSTEM_LOGGER=OFF \
-DUAGENT_USE_SYSTEM_FASTDDS=ON \
-DUAGENT_USE_SYSTEM_FASTCDR=ON \
--event-handlers=desktop_notification-
```

#### 5. Test

```bash
colcon test \
--pytest-args -s -v \
--event-handlers console_cohesion+ desktop_notification- \
--packages-select ardupilot_dds_tests
```

## Install Docker

#### 0. Build the image and run the container

Clone the ArduPilot docker project:

```bash
git clone https://github.com/ArduPilot/ardupilot_dev_docker.git
```

Build the container:

```bash
cd ~/ardupilot_dev_docker/docker
docker build -t ardupilot/ardupilot-dev-ros -f Dockerfile_dev-ros .
```

Start the container in interactive mode:

```bash
docker run -it --name ardupilot-dds ardupilot/ardupilot-dev-ros
```

Connect another bash process to the running container:

```bash
docker container exec -it ardupilot-dds /bin/bash
```

The remaining steps 1 - 5 are the same as for Ubuntu. You may need to
install MAVProxy if it is not available on the container.


```bash
python3 -m pip install -U MAVProxy
```


## Test details

The launch file replicates the following commands:

```bash
socat -d -d pty,raw,echo=0,link=./dev/ttyROS0 pty,raw,echo=0,link=./dev/ttyROS1
```

```bash
ros2 run micro_ros_agent micro_ros_agent serial --baudrate 115200 --dev ./dev/ttyROS0
```

```bash
arducopter --synthetic-clock --wipe --model quad --speedup 1 --slave 0 --instance 0 --serial1 uart:./dev/ttyROS1 --defaults $(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm --sim-address 127.0.0.1
```

```bash
mavproxy.py --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501
```

Using individual launch files

```bash
ros2 launch ardupilot_sitl virtual_ports.launch.py tty0:=./dev/ttyROS0 tty1:=./dev/ttyROS1
```

```bash
ros2 launch ardupilot_sitl micro_ros_agent.launch.py transport:=serial baudrate:=115200 device:=./dev/ttyROS0
```

```bash
ros2 launch ardupilot_sitl sitl.launch.py synthetic_clock:=True wipe:=True model:=quad speedup:=1 slave:=0 instance:=0 serial1:=uart:./dev/ttyROS1 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm sim_address:=127.0.0.1
```

```bash
ros2 launch ardupilot_sitl mavproxy.launch.py master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```

Using combined launch file

```bash
ros2 launch ardupilot_sitl sitl_dds_serial.launch.py \
\
tty0:=./dev/ttyROS0 \
tty1:=./dev/ttyROS1 \
\
transport:=serial \
baudrate:=115200 \
device:=./dev/ttyROS0 \
\
synthetic_clock:=True \
wipe:=True \
model:=quad \
speedup:=1 \
slave:=0 \
instance:=0 \
serial1:=uart:./dev/ttyROS1 \
defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm \
sim_address:=127.0.0.1 \
\
master:=tcp:127.0.0.1:5760 \
sitl:=127.0.0.1:5501
```

UDP version

```
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```

## DDS Configuration

### Transport Options

ArduPilot DDS supports two transport mechanisms for communicating with the micro-ROS agent:

#### Serial Transport

**Use Cases**:
- Hardware deployments with physical UART connections
- Platforms where network interfaces are unavailable or restricted
- Development with USB-serial adapters

**Performance Characteristics**:
- Typical baudrate: 115200 bps (configurable up to 921600 bps on supported hardware)
- Lower latency than UDP on direct connections
- No network configuration required
- Limited by serial port bandwidth

**Configuration Parameters**:
- `SERIAL1_PROTOCOL = 45` - Enables DDS on the serial port (see `AP_SerialManager.h` for protocol definitions)
- `SERIAL1_BAUD = 115` - Sets baudrate to 115200 (value is in hundred-baud increments)
- `DDS_ENABLE = 1` - Enables the DDS subsystem

**Micro-ROS Agent Command**:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --baudrate 115200 --dev /dev/ttyUSB0
```

#### UDP Transport

**Use Cases**:
- SITL (Software-In-The-Loop) simulation - recommended for development
- Network-based hardware deployments
- Multi-agent scenarios requiring flexible networking

**Performance Characteristics**:
- Higher bandwidth than serial (limited by network, not physical interface)
- Suitable for high-frequency sensor data streaming
- Supports multiple simultaneous connections
- May experience packet loss on congested networks

**Configuration Parameters**:
- `DDS_ENABLE = 1` - Enables the DDS subsystem
- `DDS_UDP_PORT = 2019` - Default UDP port (configurable in `AP_DDS_config.h`)
- IP address configuration handled by network stack

**Micro-ROS Agent Command**:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
```

For UDP6 (IPv6):
```bash
ros2 run micro_ros_agent micro_ros_agent udp6 --port 2019
```

### Quality of Service (QoS) Policies

DDS and ROS 2 use QoS policies to control message delivery behavior. Understanding and configuring QoS correctly is essential for reliable communication.

#### Reliability

**BEST_EFFORT**:
- Messages may be lost if network is congested
- Lower latency, higher throughput
- Suitable for high-frequency sensor data where occasional loss is acceptable
- Example topics: `/ap/imu/experimental/data`, `/ap/geopose/filtered`

**RELIABLE**:
- Guarantees message delivery with acknowledgments and retransmission
- Higher latency due to acknowledgment overhead
- Suitable for commands and critical state information
- Example topics: Service requests/responses, `/ap/cmd_vel`

#### Durability

**VOLATILE**:
- Messages exist only while publisher is active
- New subscribers only receive messages published after subscription
- Lower memory overhead
- Suitable for streaming sensor data

**TRANSIENT_LOCAL**:
- Last message is retained and delivered to late-joining subscribers
- Essential for configuration data and static transforms
- Example topics: `/ap/tf_static`

#### Example QoS Configuration in Python

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        
        # Configure QoS for high-frequency sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Accept message loss
            durability=DurabilityPolicy.VOLATILE,        # No history retention
            depth=1                                       # Queue size
        )
        
        self.subscription = self.create_subscription(
            TwistStamped,
            '/ap/twist/filtered',
            self.velocity_callback,
            qos_profile
        )
    
    def velocity_callback(self, msg):
        self.get_logger().info(f'Velocity: {msg.twist.linear.x:.2f} m/s')
```

For static transforms, use TRANSIENT_LOCAL durability:
```python
from tf2_msgs.msg import TFMessage

qos_static_tf = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

subscription = self.create_subscription(
    TFMessage,
    '/ap/tf_static',
    self.static_tf_callback,
    qos_static_tf
)
```

### Topic and Service Discovery

#### Agent Registration

When the micro-ROS agent starts, it listens for DDS participant announcements:

1. ArduPilot's `AP_DDS` library initializes the Micro XRCE-DDS client
2. Client sends CREATE_SESSION request to agent
3. Agent responds with session ID
4. Client registers topics and services with agent
5. Agent bridges DDS data to ROS 2 domain

**Typical Discovery Timeline**:
- Session establishment: ~100-500ms
- Topic registration: ~50-200ms per topic
- Full discovery with 15+ topics: ~2-5 seconds

#### Participant Discovery

ROS 2 nodes can discover ArduPilot topics using standard ROS 2 discovery:

```bash
# List all nodes (should show /ardupilot_dds)
ros2 node list

# List all topics published by ArduPilot
ros2 topic list | grep "^/ap/"

# Check topic info and publishers
ros2 topic info /ap/geopose/filtered

# Monitor discovery process
ros2 daemon stop && ros2 daemon start --verbose
```

**Common Discovery Issues**:
- Participant not appearing: Check network/serial connectivity
- Topics missing: Verify DDS_ENABLE parameter and feature compilation flags
- Delayed discovery: Normal for first connection; subsequent connections faster

## ROS 2 Message Mapping

### ArduPilot to ROS 2 Topic Mapping

The following table documents the comprehensive mapping between ArduPilot internal data and published ROS 2 topics:

| ArduPilot Topic | ROS 2 Message Type | Description | Update Rate |
|-----------------|-------------------|-------------|-------------|
| `/ap/clock` | `rosgraph_msgs/msg/Clock` | Simulation or system clock for time synchronization | 50 Hz |
| `/ap/time` | `builtin_interfaces/msg/Time` | Current autopilot time (sec, nanosec) | 50 Hz |
| `/ap/geopose/filtered` | `geographic_msgs/msg/GeoPoseStamped` | Global position and orientation (GPS + attitude) | 10 Hz |
| `/ap/pose/filtered` | `geometry_msgs/msg/PoseStamped` | Local position and orientation relative to origin | 10 Hz |
| `/ap/twist/filtered` | `geometry_msgs/msg/TwistStamped` | Linear and angular velocity in body frame | 10 Hz |
| `/ap/gps_global_origin/filtered` | `geographic_msgs/msg/GeoPointStamped` | Global origin for local coordinate system | On change |
| `/ap/navsat` | `sensor_msgs/msg/NavSatFix` | Raw GPS data (lat, lon, alt) with status and covariance | 5 Hz |
| `/ap/imu/experimental/data` | `sensor_msgs/msg/Imu` | IMU data (linear accel, angular velocity, orientation) | 50-100 Hz |
| `/ap/battery` | `sensor_msgs/msg/BatteryState` | Battery voltage, current, charge percentage | 1 Hz |
| `/ap/airspeed` | `ardupilot_msgs/msg/Airspeed` | Airspeed sensor data (indicated and true airspeed) | 5 Hz |
| `/ap/rc` | `ardupilot_msgs/msg/Rc` | RC receiver channel values | 10 Hz |
| `/ap/status` | `ardupilot_msgs/msg/Status` | Flight mode, armed state, system status flags | 1 Hz |
| `/ap/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms for sensor mounting positions | On startup |

**Subscribed Topics** (Commands from ROS 2 to ArduPilot):

| ROS 2 Topic | Message Type | Description | ArduPilot Action |
|-------------|--------------|-------------|------------------|
| `/ap/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity command in body frame | Sets velocity targets in GUIDED mode |
| `/ap/cmd_gps_pose` | `ardupilot_msgs/msg/GlobalPosition` | GPS waypoint command | Flies to GPS position in GUIDED mode |
| `/ap/joy` | `sensor_msgs/msg/Joy` | Joystick/RC override | Overrides RC input channels (max 8 channels) |
| `/ap/tf` | `tf2_msgs/msg/TFMessage` | External transforms | Used for visual odometry integration |

### Coordinate Frame Conversions

ArduPilot and ROS 2 use different coordinate frame conventions. The `AP_DDS` library performs automatic conversions per [REP-147](https://www.ros.org/reps/rep-0147.html).

#### ArduPilot Native: NED (North-East-Down)

```
     North (X forward)
       ↑
       |
       |
       +------→ East (Y right)
      /
     /
    ↓
  Down (Z down)
```

- X-axis: Points North
- Y-axis: Points East  
- Z-axis: Points Down
- Right-handed coordinate system
- Used in `AP_AHRS`, `AP_NavEKF`, and all internal navigation

#### ROS 2 Standard: ENU (East-North-Up)

```
       Up (Z up)
       ↑
       |
       |
       +------→ East (X forward)
      /
     /
    ↓
  North (Y left)
```

- X-axis: Points East
- Y-axis: Points North
- Z-axis: Points Up
- Right-handed coordinate system
- Required by ROS 2 REP-105 for `map` and `odom` frames

#### Transformation Details

The transformation from ArduPilot NED to ROS 2 ENU involves:

**Position Vector Transformation**:
```
ROS2_ENU.x = ArduPilot_NED.y  (East)
ROS2_ENU.y = ArduPilot_NED.x  (North)
ROS2_ENU.z = -ArduPilot_NED.z (Up = -Down)
```

**Quaternion Rotation**:
The orientation quaternion requires a 90° rotation about the Z-axis:
```python
# Pseudo-code for NED to ENU quaternion conversion
def ned_to_enu_quaternion(q_ned):
    # Rotate 90° about Z-axis (yaw rotation)
    # This swaps X/Y and adjusts signs
    q_enu = Quaternion(
        x=q_ned.y,
        y=q_ned.x,
        z=-q_ned.z,
        w=q_ned.w
    )
    return q_enu
```

**Velocity Vector Transformation**:
Same as position (linear velocities):
```
twist_enu.linear.x = twist_ned.linear.y
twist_enu.linear.y = twist_ned.linear.x
twist_enu.linear.z = -twist_ned.linear.z
```

Angular velocities (body frame rates) also transform:
```
twist_enu.angular.x = twist_ned.angular.y  (roll rate)
twist_enu.angular.y = twist_ned.angular.x  (pitch rate)
twist_enu.angular.z = -twist_ned.angular.z (yaw rate)
```

**Implementation Reference**: 
Source: `libraries/AP_DDS/AP_DDS_External_Odom.cpp` contains production coordinate transformation code.

#### Body Frame vs Earth Frame

Both ArduPilot and ROS 2 use:
- **Body Frame**: Origin at vehicle center of mass, axes aligned with vehicle (forward-right-down for ArduPilot, forward-left-up for ROS 2)
- **Earth Frame**: Fixed reference frame (NED for ArduPilot, ENU for ROS 2)

Messages specify their reference frame in the `frame_id` field:
- `base_link`: Vehicle body frame (ROS 2 convention, forward-left-up)
- `base_link_ned`: Vehicle body frame (ArduPilot convention, forward-right-down)
- `map`: Local earth-fixed frame (ENU per REP-105)

### Time Synchronization

#### Clock Topics

ArduPilot publishes two time-related topics:

**`/ap/clock`** (`rosgraph_msgs/msg/Clock`):
- Used for ROS 2 simulation time synchronization
- Essential for SITL with `--synthetic-clock` option
- ROS 2 nodes with `use_sim_time:=true` parameter synchronize to this clock
- Enables deterministic replay and time control in simulation

**`/ap/time`** (`builtin_interfaces/msg/Time`):
- ArduPilot's internal timestamp (microseconds since boot)
- Useful for correlating ArduPilot logs with ROS 2 bag files
- Monotonic clock, not affected by GPS time jumps

#### Using Simulation Time in SITL

Launch SITL with synthetic clock:
```bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py synthetic_clock:=True
```

Launch ROS 2 nodes with sim time enabled:
```bash
ros2 run my_package my_node --ros-args --param use_sim_time:=true
```

Verify sim time is active:
```bash
# Should show "use_sim_time: true"
ros2 param get /my_node use_sim_time
```

#### Hardware Time Sync

On real hardware, ArduPilot's clock is independent:
- `use_sim_time:=false` (default for hardware)
- ROS 2 nodes use system clock
- Timestamp correlation via `header.stamp` in messages

## Troubleshooting

### DDS Agent Connectivity Issues

#### Symptom: "micro_ros_agent not connecting" or "Session establishment timeout"

**Possible Causes and Solutions**:

1. **Incorrect transport configuration**:
   - Serial: Verify baudrate matches on both sides (`SERIAL1_BAUD` parameter and agent `--baudrate`)
   - UDP: Check port number (default 2019) and IP address
   - Solution: Double-check parameters and restart both agent and ArduPilot

2. **DDS not enabled**:
   - Check `DDS_ENABLE` parameter: `param show DDS_ENABLE`
   - Solution: `param set DDS_ENABLE 1` and reboot ArduPilot

3. **Wrong serial protocol**:
   - Verify serial port is configured for DDS: `param show SERIAL1_PROTOCOL`
   - Solution: `param set SERIAL1_PROTOCOL 45` (45 = DDS protocol ID)

4. **Agent not running or wrong device**:
   - Verify agent command matches transport type
   - Solution for serial: Check device path exists: `ls -l /dev/ttyUSB0`
   - Solution for UDP: Verify nothing else is using port 2019: `sudo netstat -tulpn | grep 2019`

**Debugging Commands**:

Test serial connectivity with miniterm:
```bash
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200 --echo --encoding hexlify
# Should see XRCE-DDS binary data stream if DDS is active
```

Check UDP connectivity:
```bash
# On agent machine, listen for UDP packets
sudo tcpdump -i any -n udp port 2019
# Should see packets when ArduPilot is running with DDS enabled
```

Enable verbose agent logging:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --baudrate 115200 --dev /dev/ttyUSB0 --verbose 6
```

### Serial Port Permission Problems

#### Symptom: "Permission denied: /dev/ttyUSB0" or "Could not open serial device"

**Solution 1: Add user to dialout group** (recommended for development):
```bash
sudo usermod -aG dialout $USER
# Log out and log back in for group change to take effect
```

Verify group membership:
```bash
groups
# Should include 'dialout'
```

**Solution 2: Create udev rule** (recommended for production):
```bash
# Create rule file to automatically set permissions
sudo tee /etc/udev/rules.d/99-ardupilot.rules > /dev/null << 'EOF'
# ArduPilot USB serial devices
SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", ATTRS{idProduct}=="*", MODE="0666", GROUP="plugdev"
# FTDI devices commonly used with ArduPilot
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="plugdev"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reconnect USB device
```

**Solution 3: Temporary permission change** (for testing only):
```bash
sudo chmod 666 /dev/ttyUSB0
# WARNING: Permissions reset on reboot or device reconnection
```

**Find device by ID** (more reliable than /dev/ttyUSB*):
```bash
# List devices with ID
ls -l /dev/serial/by-id/

# Use stable device path
ros2 run micro_ros_agent micro_ros_agent serial --baudrate 115200 \
  --dev /dev/serial/by-id/usb-ArduPilot_Pixhawk6X_210028000151323131373139-if02
```

### UDP Firewall Configuration

#### Symptom: UDP agent starts but no data received

**Check if firewall is blocking UDP port 2019**:
```bash
sudo iptables -L -n | grep 2019
```

**Solution: Allow DDS UDP port** (Linux with ufw):
```bash
sudo ufw allow 2019/udp
sudo ufw reload
sudo ufw status
```

**Solution: Allow DDS UDP port** (Linux with iptables):
```bash
sudo iptables -A INPUT -p udp --dport 2019 -j ACCEPT
sudo iptables -A OUTPUT -p udp --sport 2019 -j ACCEPT

# Make persistent (Ubuntu/Debian)
sudo netfilter-persistent save
```

**For SITL on localhost**, firewall typically not an issue, but verify loopback works:
```bash
ping 127.0.0.1
# Should respond immediately
```

### Topic Discovery Failures

#### Symptom: "ros2 topic list" doesn't show /ap/* topics

**Diagnostic Steps**:

1. **Verify agent is running**:
   ```bash
   ps aux | grep micro_ros_agent
   ```

2. **Check ROS 2 daemon**:
   ```bash
   # Restart daemon to refresh discovery
   ros2 daemon stop
   ros2 daemon start
   ros2 topic list
   ```

3. **Verify ArduPilot DDS is active**:
   ```bash
   # In MAVProxy or other GCS
   param show DDS_ENABLE
   # Should show: DDS_ENABLE = 1.0
   ```

4. **Check ROS_DOMAIN_ID** (must match between agent and nodes):
   ```bash
   echo $ROS_DOMAIN_ID
   # Default is 0; if set differently, must match everywhere
   ```

5. **Inspect agent output**:
   - Look for "Successfully created participant" message
   - Check for topic registration confirmations
   - Agent should show "Status: running" for active topics

6. **Test with echo on specific topic**:
   ```bash
   # Try known topic with proper QoS
   ros2 topic echo /ap/time --qos-reliability best_effort
   ```

**Enable DDS discovery logging**:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/custom_profile.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

### Message Deserialization Errors

#### Symptom: "Could not deserialize message" or type mismatch errors

**Common Causes**:

1. **ROS 2 / ardupilot_msgs version mismatch**:
   - ArduPilot's IDL definitions must match compiled `ardupilot_msgs` package
   - Solution: Rebuild ROS 2 workspace after pulling latest ArduPilot:
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build --cmake-args -DBUILD_TESTING=ON
   source install/setup.bash
   ```

2. **Standard message version incompatibility**:
   - Ensure ROS 2 Humble or later (Foxy and Galactic have different message definitions)
   - Solution: Verify ROS version: `echo $ROS_DISTRO`

3. **QoS mismatch** (not technically deserialization, but similar symptoms):
   - Publisher uses RELIABLE, subscriber uses BEST_EFFORT: May work
   - Publisher uses BEST_EFFORT, subscriber uses RELIABLE: Won't connect
   - Solution: Match QoS policies or use compatible settings (see QoS section above)

**Debugging Type Mismatches**:

Check message type hash:
```bash
ros2 topic info /ap/geopose/filtered --verbose
# Shows type, QoS, and publishers/subscribers
```

Inspect message definition:
```bash
ros2 interface show geographic_msgs/msg/GeoPoseStamped
```

Compare with ArduPilot's IDL:
```bash
cat libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPoseStamped.idl
```

### Common Error Messages

| Error Message | Cause | Solution |
|---------------|-------|----------|
| `failed to create session` | Agent not reachable or transport mismatch | Check transport configuration and network/serial connectivity |
| `Serial port not found` | Incorrect device path or permissions | Verify device with `ls /dev/ttyUSB*`, check permissions |
| `Address already in use` | Another process using UDP port 2019 | Find process with `lsof -i :2019` and terminate it |
| `No publishers found` | Topic not yet registered or DDS disabled | Wait a few seconds for discovery, verify DDS_ENABLE=1 |
| `QoS mismatch` | Incompatible QoS between publisher and subscriber | Match QoS profiles (see QoS section) |
| `Lost connection to agent` | Network interruption or serial disconnect | Check physical connection, restart agent |
| `Timeout waiting for service` | Service not available or discovery incomplete | Verify ArduPilot is armed/ready, increase timeout |

## Example ROS 2 Applications

This section provides practical examples for integrating ROS 2 with ArduPilot via DDS.

### Sample ROS 2 Subscriber Node

Example Python subscriber for GPS position data with proper QoS configuration:

```python
#!/usr/bin/env python3
"""
Example ROS 2 subscriber node for ArduPilot GPS position.
Demonstrates proper QoS configuration and coordinate frame handling.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geographic_msgs.msg import GeoPoseStamped


class ArduPilotPositionSubscriber(Node):
    """Subscribe to ArduPilot global position and orientation."""
    
    def __init__(self):
        super().__init__('ardupilot_position_subscriber')
        
        # Configure QoS to match ArduPilot publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to filtered global pose topic
        self.subscription = self.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self.geopose_callback,
            qos_profile
        )
        
        self.get_logger().info('ArduPilot position subscriber initialized')
    
    def geopose_callback(self, msg: GeoPoseStamped):
        """Process received GPS position and orientation."""
        # Extract position
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        alt = msg.pose.position.altitude
        
        # Extract orientation (quaternion in ENU frame per ROS 2 convention)
        orientation = msg.pose.orientation
        
        self.get_logger().info(
            f'Position: lat={lat:.6f}°, lon={lon:.6f}°, alt={alt:.1f}m | '
            f'Orientation: x={orientation.x:.3f}, y={orientation.y:.3f}, '
            f'z={orientation.z:.3f}, w={orientation.w:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotPositionSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Save as `ardupilot_position_subscriber.py`, make executable, and run:
```bash
chmod +x ardupilot_position_subscriber.py
./ardupilot_position_subscriber.py
```

### Sample ROS 2 Publisher Node (Command/Control)

Example Python publisher for sending velocity commands to ArduPilot:

```python
#!/usr/bin/env python3
"""
Example ROS 2 publisher node for sending velocity commands to ArduPilot.
Requires vehicle in GUIDED mode.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math


class ArduPilotVelocityCommander(Node):
    """Publish velocity commands to ArduPilot in GUIDED mode."""
    
    def __init__(self):
        super().__init__('ardupilot_velocity_commander')
        
        # Publisher for velocity commands
        # Note: No special QoS needed for command topics (uses default RELIABLE)
        self.publisher = self.create_publisher(
            TwistStamped,
            '/ap/cmd_vel',
            10
        )
        
        # Timer to send periodic commands
        self.timer = self.create_timer(0.1, self.publish_velocity_command)  # 10 Hz
        self.angle = 0.0
        
        self.get_logger().info('ArduPilot velocity commander initialized')
        self.get_logger().info('Ensure vehicle is in GUIDED mode!')
    
    def publish_velocity_command(self):
        """Publish circular velocity pattern (for demonstration)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Circular pattern: constant forward, sinusoidal lateral
        # Velocities in body frame (ENU convention: x=forward, y=left, z=up)
        msg.twist.linear.x = 2.0  # 2 m/s forward
        msg.twist.linear.y = math.sin(self.angle) * 0.5  # 0.5 m/s lateral oscillation
        msg.twist.linear.z = 0.0  # No vertical velocity
        
        # No rotation
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.publisher.publish(msg)
        self.angle += 0.1  # Increment for next iteration


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotVelocityCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send zero velocity on exit
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        node.publisher.publish(msg)
        node.get_logger().info('Sent zero velocity command')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### TF (Transform) Tree Visualization

ArduPilot publishes static transforms for sensor mounting positions on `/ap/tf_static`.

**View transform tree**:
```bash
# Install tf2_tools if not already installed
sudo apt install ros-humble-tf2-tools

# Generate PDF of transform tree
ros2 run tf2_tools view_frames

# Opens evince with frames.pdf showing complete TF tree
```

**Listen to transforms programmatically**:
```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer


class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.lookup_transform)
    
    def lookup_transform(self):
        try:
            # Look up transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            self.get_logger().info(f'Transform: {transform}')
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
```

### Integration with ROS 2 Navigation Stack (Nav2)

ArduPilot can integrate with the ROS 2 Nav2 navigation stack for advanced path planning:

**Prerequisites**:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

**Coordinate Frame Setup**:

Nav2 requires these frames:
- `map`: Fixed world frame (ENU convention)
- `odom`: Odometry frame (ENU convention)  
- `base_link`: Robot/vehicle body frame

ArduPilot publishes:
- `/ap/geopose/filtered` → Can be used as `map` → `base_link`
- `/ap/pose/filtered` → Local pose relative to origin (use as `odom` → `base_link`)

**Example integration approach**:

1. **Publish TF from ArduPilot pose**:
   ```python
   from geometry_msgs.msg import PoseStamped
   from tf2_ros import TransformBroadcaster
   import tf_transformations
   
   class ArduPilotTFBroadcaster(Node):
       def __init__(self):
           super().__init__('ardupilot_tf_broadcaster')
           self.tf_broadcaster = TransformBroadcaster(self)
           
           qos = QoSProfile(
               reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,
               depth=1
           )
           
           self.subscription = self.create_subscription(
               PoseStamped,
               '/ap/pose/filtered',
               self.pose_callback,
               qos
           )
       
       def pose_callback(self, msg):
           # Broadcast odom -> base_link transform
           t = TransformStamped()
           t.header.stamp = msg.header.stamp
           t.header.frame_id = 'odom'
           t.child_frame_id = 'base_link'
           
           t.transform.translation.x = msg.pose.position.x
           t.transform.translation.y = msg.pose.position.y
           t.transform.translation.z = msg.pose.position.z
           
           t.transform.rotation = msg.pose.orientation
           
           self.tf_broadcaster.sendTransform(t)
   ```

2. **Send Nav2 goals to ArduPilot**:
   - Nav2 publishes cmd_vel on `/cmd_vel`
   - Remap or bridge to `/ap/cmd_vel` for ArduPilot
   - Ensure vehicle is in GUIDED mode

3. **Configure Nav2 for ArduPilot constraints**:
   - Adjust velocity limits in Nav2 config to match vehicle capabilities
   - Configure costmap for obstacle avoidance (requires additional sensors)
   - Set appropriate controller parameters for vehicle dynamics

### Reference Example Nodes

The `ardupilot_dds_tests` package contains additional example nodes:

**Time Listener** (simple subscriber):
```bash
ros2 run ardupilot_dds_tests time_listener
```
Source: `Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/time_listener.py`

**Pre-arm Check Service** (service client):
```bash
ros2 run ardupilot_dds_tests pre_arm_check_service
```
Source: `Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/pre_arm_check_service.py`

**Copter Takeoff** (complete mission sequence):
```bash
ros2 run ardupilot_dds_tests copter_takeoff
```
Source: `Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/copter_takeoff.py`
- Demonstrates service calls (arm, mode switch, takeoff)
- Subscribes to position feedback
- Complete state machine for autonomous takeoff

**Plane Waypoint Follower** (waypoint mission):
```bash
ros2 run ardupilot_dds_tests plane_waypoint_follower
```
Source: `Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/plane_waypoint_follower.py`
- Publishes GPS waypoint commands
- Monitors mission progress
- Example of GPS-based navigation

**Usage Pattern** from example nodes:
1. Initialize ROS 2 node
2. Create service clients with `wait_for_service()`
3. Configure QoS profiles for subscriptions (typically BEST_EFFORT for sensor data)
4. Implement callbacks for data processing
5. Use `rclpy.spin()` or `spin_once()` for event handling
6. Clean shutdown with `destroy_node()` and `rclpy.shutdown()`

## Additional Resources

- **ArduPilot DDS Documentation**: See `libraries/AP_DDS/README.md` for lower-level DDS architecture
- **ROS 2 Interface Reference**: [https://ardupilot.org/dev/docs/ros2-interfaces.html](https://ardupilot.org/dev/docs/ros2-interfaces.html)
- **Micro-ROS Agent**: [https://github.com/micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
- **REP-147 (Coordinate Frames)**: [https://www.ros.org/reps/rep-0147.html](https://www.ros.org/reps/rep-0147.html)
- **REP-105 (TF Conventions)**: [https://www.ros.org/reps/rep-0105.html](https://www.ros.org/reps/rep-0105.html)
