# Example 5 — Hardware Connection and Hover Test

This is the first hardware example. A real Crazyflie drone is connected via Crazyradio, all pre-flight conditions are verified (mocap pairing, EKF convergence, sensor self-test, battery), a brief hover is executed, and the drone lands safely. The drone must be physically placed facing world +X before takeoff — single-marker Vicon provides position only (no orientation), so yaw is unobservable and cannot be corrected automatically.

The flight is intentionally minimal — takeoff to 30cm, hover 5 seconds, land — because the real lesson is the **pre-flight safety workflow**. The keyboard-gated state machine requires explicit keypress approval at each transition point. The script never proceeds autonomously from one flight phase to the next.

This example assumes completion of Examples 1-4 and that all hardware prerequisites (Part 0, Sections 1(c), 1(d), and 2) are set up.

---

## At a glance

| Item | Value |
|---|---|
| Goal | First hardware hover — connect, verify, hover, land safely |
| Requires hardware? | **Yes** — Crazyflie, Crazyradio, Vicon |
| Main package | `hardware_hover` |
| Main script | `hover_test.py` |
| New concepts | Keyboard-gated safety state machine, pre-flight checks (mocap pairing, EKF convergence, CAN_FLY), CSV logging, raw terminal input |
| Expected result | Drone takes off to 30cm after user presses 'T', hovers, lands safely |

---

## Contents

1. [Files Created](#files-created)
2. [Section 1: Create the Package](#section-1-create-the-package)
3. [Section 2: Write crazyflies.yaml](#section-2-write-crazyflieyaml)
4. [Section 3: Write motion_capture.yaml](#section-3-write-motion_captureyaml)
5. [Section 4: Write flight_config.yaml](#section-4-write-flight_configyaml)
6. [Section 5: Write the Launch File](#section-5-write-the-launch-file)
7. [Section 6: Write hover_test.py](#section-6-write-hover_testpy)
8. [Section 7: Register Entry Points](#section-7-register-entry-points)
9. [Section 8: Build and Test](#section-8-build-and-test)
10. [Potential Issues](#potential-issues)

---

## Files Created

```
tutorial_ws/src/hardware_hover/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   ├── crazyflies.yaml
│   ├── motion_capture.yaml
│   └── flight_config.yaml
├── launch/
│   └── hardware_hover.launch.py
└── hardware_hover/
    ├── __init__.py
    └── hover_test.py
```

---

## Section 1: Create the Package

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws/src
ros2 pkg create hardware_hover --build-type ament_python \
    --dependencies rclpy crazyflie_interfaces motion_capture_tracking_interfaces geometry_msgs std_srvs
cd hardware_hover
mkdir -p config launch resource
touch resource/hardware_hover
```

The dependency `motion_capture_tracking_interfaces` is needed because the script subscribes to `/poses` (NamedPoseArray) to detect mocap pairing. This is the first example requiring this dependency — the sim examples only used `crazyflie_interfaces`.

---

## Section 2: Write crazyflies.yaml

Create `config/crazyflies.yaml`:

```bash
touch config/crazyflies.yaml
```

This is the first hardware drone configuration. Key differences from the sim configs: a real radio URI instead of `sim://`, motion capture enabled with `librigidbodytracker` for single-marker tracking, and PID controller instead of Mellinger.

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701
    initial_position: [0.0, 0.0, 0.0]
    type: cf21

robot_types:
  cf21:
    motion_capture:
      enabled: true
      tracking: "librigidbodytracker"
      marker: default_single_marker
      dynamics: default
    big_quad: false
    battery:
      voltage_warning: 3.8
      voltage_critical: 3.7

all:
  firmware_params:
    commander:
      enHighLevel: 1
    stabilizer:
      estimator: 2
      controller: 1
    locSrv:
      extPosStdDev: 1e-3
      extQuatStdDev: 0.5e-1
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10  # Hz
      status:
        frequency: 1   # Hz
```

`firmware_logging: enabled: true` enables the cflib server's logging subsystem. The `default_topics` block tells the server which log variables to request from the drone. `pose: frequency: 10` requests the EKF state estimate at 10 Hz — this data is published as `/{drone}/pose` (PoseStamped) and is **required** by the script for EKF convergence monitoring, position feedback, and safety timeout checks. Without it, the WAITING_FOR_EKF state aborts before flight. `status: frequency: 1` requests battery voltage and supervisor flags at 1 Hz (used for low-battery and tumble detection).

**Field-by-field explanation of hardware-specific settings:**

`uri: radio://0/80/2M/E7E7E7E701` — a real Crazyradio URI. The format is `radio://<dongle_index>/<channel>/<datarate>/<address>`. `0` selects the first (or only) Crazyradio dongle. `80` is the radio channel (0–125). `2M` is the 2 Mbps datarate. `E7E7E7E701` is the drone's unique address configured in Section 1(d) of the prerequisites. Each drone must have a distinct address.

`motion_capture: enabled: true` — mocap is now active. `tracking: "librigidbodytracker"` tells CS2 to use its own frame-by-frame rigid body tracking from raw unlabeled marker positions. This is required because Vicon Tracker is configured for unordered markers (single small markers that cannot be labeled). `marker: default_single_marker` references a single-point marker configuration defined in `motion_capture.yaml` (Section 3).

`stabilizer: controller: 1` — PID controller. In the sim examples, `controller: 2` (Mellinger) was used for best-case tracking performance. On hardware, PID is safer: it is simpler, more robust to sensor noise (real Vicon data is noisier than simulation ground truth), and less sensitive to velocity estimate errors from the EKF.

**Controller note:** The `cmd_full_state` message format, topic, 30Hz rate, and field meanings are identical regardless of whether PID or Mellinger is running. The script publishes the same setpoints either way. Only the internal tracking performance differs.

---

## Section 3: Write motion_capture.yaml

Create `config/motion_capture.yaml`:

```bash
touch config/motion_capture.yaml
```

This configures `motion_capture_tracking_node` — the connection to Vicon Tracker and the marker-to-drone assignment rules.

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "vicon"
    hostname: "192.168.10.1"
    port: 801
    marker_configurations:
      default_single_marker:
        offset: [0.0, 0.0, 0.0]
        points:
          p0: [0.0, 0.0, 0.0]
    dynamics_configurations:
      default:
        max_velocity: [2.0, 2.0, 3.0]
        max_angular_velocity: [20.0, 20.0, 10.0]
        max_roll: 1.4
        max_pitch: 1.4
        max_fitness_score: 0.001
    topics:
      frame_id: "world"
      poses:
        qos:
          mode: "sensor"
          deadline: 100.0  # Hz
```

`type: "vicon"` selects the Vicon DataStream SDK backend (compiled into the `motion_capture_tracking` package — no external SDK installation needed). `hostname: "192.168.10.1"` is the Vicon Tracker PC IP address configured in the mocap prerequisites. `port: 801` is the default Vicon DataStream TCP port.

`marker_configurations.default_single_marker` defines a single point at (0, 0, 0) — the drone's center. `librigidbodytracker` looks for a single marker at this position relative to the drone. `offset: [0.0, 0.0, 0.0]` means the marker is at the drone's center (no offset).

`dynamics_configurations.default` sets physical limits for the tracker's motion model. `max_velocity: [2.0, 2.0, 3.0]` are per-axis limits in m/s (X, Y, Z); `max_angular_velocity: [20.0, 20.0, 10.0]` are per-axis limits in rad/s (roll, pitch, yaw); `max_roll: 1.4` and `max_pitch: 1.4` are absolute angle limits in radians; `max_fitness_score: 0.001` is the maximum acceptable marker-fit error. The tracker uses these to predict where each drone should be in the next frame, improving marker assignment.

`topics.poses.qos.mode: "sensor"` selects a pre-defined QoS profile for sensor data (BEST_EFFORT reliability, VOLATILE durability, KEEP_LAST history). This is required — without it, the C++ node falls back to RELIABLE + Infinite deadline, which will never match the `crazyflie_server` subscriber, and no messages will flow. `deadline: 100.0` sets the expected rate in Hz (100 Hz → 10 ms deadline). The CS2 launch file passes this same value to `crazyflie_server` via `poses_qos_deadline` so both endpoints agree. Both values must be floats (not integers) to satisfy the C++ ROS2 parameter type system.

---

## Section 4: Write flight_config.yaml

Create `config/flight_config.yaml`:

```bash
touch config/flight_config.yaml
```

Flight parameters and safety bounds are separated from the drone definition so they can be tuned without modifying the radio and mocap configuration.

```yaml
flight:
  drone_name: "cf1"
  height_m: 0.30
  hover_duration_s: 5.0

safety:
  max_distance_m: 1.0
  max_height_m: 0.60
  min_height_m: 0.05
  pose_timeout_s: 0.5
  battery_critical_v: 3.2

control:
  rate_hz: 30
  takeoff_duration_s: 3.0
  land_duration_s: 3.0
  hover_stabilize_s: 3.0
```

`flight.height_m: 0.30` — hover height. 30cm is conservative for a first flight.

`safety.max_distance_m: 1.0` — maximum horizontal distance from the origin before emergency landing. At 30cm height with a 1.0m radius, the drone operates within a safe cylinder.

`safety.max_height_m: 0.60` and `min_height_m: 0.05` — vertical limits. Exceeding either triggers emergency landing.

`safety.pose_timeout_s: 0.5` — if no EKF pose data arrives for 0.5 seconds, something is wrong (mocap failure, radio disconnection, or `crazyflie_server` crash).

`safety.battery_critical_v: 3.2` — below this voltage, the battery is near depletion and continued flight risks sudden power loss.

`control.hover_stabilize_s: 3.0` — after takeoff, the High-Level Commander holds position for 3 seconds. This gives the EKF time to settle after the altitude change.

---

## Section 5: Write the Launch File

Create `launch/hardware_hover.launch.py`:

```bash
touch launch/hardware_hover.launch.py
```

This launch file starts the full hardware pipeline: `motion_capture_tracking_node` (connects to Vicon), `crazyflie_server` (connects to the drone via Crazyradio), and optionally the viewer from Example 4.

```python
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hardware_hover')
    cs2_share = get_package_share_directory('crazyflie')

    # Paths to config files
    crazyflies_path = os.path.join(pkg_dir, 'config', 'crazyflies.yaml')
    mocap_yaml_path = os.path.join(pkg_dir, 'config', 'motion_capture.yaml')
    server_yaml_path = os.path.join(cs2_share, 'config', 'server.yaml')
    urdf_path = os.path.join(cs2_share, 'urdf', 'crazyflie_description.urdf')

    # Load configs
    with open(crazyflies_path) as f:
        crazyflies = yaml.safe_load(f)
    with open(mocap_yaml_path) as f:
        mocap_cfg = yaml.safe_load(f)
    with open(server_yaml_path) as f:
        server_cfg = yaml.safe_load(f)
    with open(urdf_path) as f:
        robot_desc = f.read()

    # Build mocap parameters (marker configs + Vicon connection)
    mocap_params = mocap_cfg['/motion_capture_tracking']['ros__parameters']
    mocap_params['rigid_bodies'] = {}
    for key, value in crazyflies['robots'].items():
        if value['enabled']:
            robot_type = crazyflies['robot_types'][value['type']]
            mc = robot_type['motion_capture']
            if mc['enabled'] and mc.get('tracking') == 'librigidbodytracker':
                mocap_params['rigid_bodies'][key] = {
                    'initial_position': value['initial_position'],
                    'marker': mc['marker'],
                    'dynamics': mc['dynamics'],
                }

    # Build server parameters
    server_params = [crazyflies]
    server_params.append(
        server_cfg['/crazyflie_server']['ros__parameters'])
    server_params[1]['robot_description'] = robot_desc
    server_params[1]['poses_qos_deadline'] = \
        mocap_params['topics']['poses']['qos']['deadline']

    # Nodes
    mocap_node = Node(
        package='motion_capture_tracking',
        executable='motion_capture_tracking_node',
        name='motion_capture_tracking',
        output='screen',
        parameters=[mocap_params],
    )

    server_node = Node(
        package='crazyflie',
        executable='crazyflie_server.py',
        name='crazyflie_server',
        output='screen',
        parameters=server_params,
    )

    viewer_node = Node(
        package='vicon_viewer',
        executable='drone_viewer',
        name='drone_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='false'),
        mocap_node,
        server_node,
        viewer_node,
    ])
```

`crazyflie_server.py` (Python/cflib backend) is used instead of `crazyflie_server` (C++ backend). The C++ backend has known bugs with the Crazyradio 2.0 in PA mode. The Python backend is reliable for multi-drone broadcast.

The `rigid_bodies` dictionary is constructed by iterating over enabled drones in `crazyflies.yaml`. For each drone with `motion_capture.enabled: true` and `tracking: "librigidbodytracker"`, a rigid body entry is created with the drone's initial position, marker configuration name, and dynamics model name. This tells `librigidbodytracker` which marker configurations to search for and where to expect them.

---

## Section 6: Write hover_test.py

Create `hardware_hover/hover_test.py`:

```bash
touch hardware_hover/hover_test.py
```

This is the main script. It is written in sequential blocks, each building on the previous. The state machine and safety system are the core new content.

### 6.1: Shebang, docstring, and imports

```python
#!/usr/bin/env python3
"""
Hardware hover test with keyboard-gated safety workflow.

Performs pre-flight checks (mocap pairing, EKF convergence, sensor
self-test, battery) and a keyboard-gated hover. Safety checks run
continuously during all
streaming phases. Press 'E' at any time for emergency landing.
"""

import csv
import math
import os
import select
import sys
import threading
import time
from datetime import datetime
from enum import Enum, auto

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import FullState, Status
from crazyflie_interfaces.srv import Land, NotifySetpointsStop, Takeoff
from std_srvs.srv import Empty
```

**Imports new to this example (not seen in sim examples):**
- `csv`, `datetime` — CSV flight logging
- `select` — non-blocking keyboard input via `select.select()`
- `threading` — keyboard input thread (daemon)
- `enum.Enum` — `FlightState` state machine with named states
- `PoseStamped` — EKF pose messages on `/{drone}/pose`
- `NamedPoseArray` — mocap data on `/poses` (for pairing detection)
- `Status` — drone status messages (battery voltage and supervisor_info flags)
- `Empty` — emergency stop service (immediate motor cutoff)

### 6.2: FlightState enum and RateController

```python
class FlightState(Enum):
    WAITING_FOR_PAIR = auto()
    WAITING_FOR_EKF = auto()
    WAITING_FOR_CAN_FLY = auto()
    READY_FOR_TAKEOFF = auto()
    TAKING_OFF = auto()
    STABILIZING = auto()
    READY_TO_HOVER = auto()
    HOVERING = auto()
    LANDING = auto()
    DONE = auto()
    EMERGENCY = auto()
```

`auto()` assigns sequential integer values. The actual numbers are irrelevant — the names are used for state comparison in the state machine and for recording the current state in the CSV log. The state ordering defines the valid progression: the script moves forward through states (never backward, except to EMERGENCY which can be entered from any state).

**RateController** — add this class before `FlightState` at the top of the file. Phase-locked 30Hz timer used in all prior examples:

```python
class RateController:
    """Phase-locked rate limiter with skip-on-overrun."""
    def __init__(self, period: float):
        self.period = period
        self._next_t = 0.0
        self.overruns = 0

    def start(self):
        self._next_t = time.monotonic()

    def sleep(self):
        self._next_t += self.period
        now = time.monotonic()
        if now > self._next_t:
            self.overruns += 1
            self._next_t = now + self.period
        else:
            time.sleep(self._next_t - now)
```

### 6.3: Config loader and HoverTestNode.__init__

```python
def _load_config(config_path: str) -> dict:
    """Load flight_config.yaml, with sensible defaults if not found."""
    if not config_path:
        config_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "config", "flight_config.yaml",
        )
    config_path = os.path.abspath(config_path)
    if os.path.exists(config_path):
        with open(config_path) as f:
            return yaml.safe_load(f)
    # Fallback defaults
    return {
        "flight": {
            "height_m": 0.30, "hover_duration_s": 5.0,
        },
        "safety": {
            "max_distance_m": 1.0, "max_height_m": 0.60,
            "min_height_m": 0.05, "pose_timeout_s": 0.5,
            "battery_critical_v": 3.2,
        },
        "control": {
            "rate_hz": 30, "takeoff_duration_s": 3.0,
            "land_duration_s": 3.0, "hover_stabilize_s": 3.0,
        },
    }
```

The fallback defaults let the script run even without a config file. This is useful during development — the reader can start with defaults and add a config file later for tuning.

```python
class HoverTestNode(Node):

    def __init__(self):
        super().__init__("hover_test")

        # --- Parameters (settable from command line) ---
        self.declare_parameter("drone_name", "cf1")
        self.declare_parameter("config", "")
        self._drone = (
            self.get_parameter("drone_name")
            .get_parameter_value().string_value)
        config_path = (
            self.get_parameter("config")
            .get_parameter_value().string_value)
        self._cfg = _load_config(config_path)

        # --- Extract parameters from config ---
        flight = self._cfg["flight"]
        safety = self._cfg["safety"]
        control = self._cfg["control"]

        self._height = flight["height_m"]
        self._hover_dur = flight["hover_duration_s"]

        self._max_dist = safety["max_distance_m"]
        self._max_height = safety["max_height_m"]
        self._min_height = safety["min_height_m"]
        self._pose_timeout = safety["pose_timeout_s"]
        self._batt_critical = safety["battery_critical_v"]

        self._rate_hz = control["rate_hz"]
        self._dt = 1.0 / self._rate_hz
        self._takeoff_dur = control["takeoff_duration_s"]
        self._land_dur = control["land_duration_s"]
        self._hover_stab = control["hover_stabilize_s"]
```

`declare_parameter` + `get_parameter` is the ROS2 parameter API. Parameters can be set from the command line: `--ros-args -p drone_name:=cf2 -p config:=/path/to/config.yaml`. If not set, defaults are used (drone `cf1`, empty config path which triggers the fallback).

```python
        # --- State ---
        self._state = FlightState.WAITING_FOR_PAIR
        self._current_pose = None
        self._last_pose_time = 0.0
        self._battery_v = None
        self._supervisor_info = 0
        self._paired = False
        self._airborne = False
        self._state_start_time = 0.0
        self._emergency_triggered = False
        self._emergency_requested = False
```

`_emergency_triggered` prevents duplicate emergency handling. `_emergency_requested` is set by the keyboard thread when `E` or Ctrl+C is pressed — the main loop polls it and calls `_emergency_land()` on the main thread (two threads must not spin the same rclpy node concurrently).

```python
        # --- Subscribers ---
        # EKF pose (for convergence check + safety monitoring)
        self._pose_sub = self.create_subscription(
            PoseStamped, f"/{self._drone}/pose",
            self._pose_cb, 10)

        # Drone status (battery + supervisor_info)
        self._status_sub = self.create_subscription(
            Status, f"/{self._drone}/status",
            self._status_cb, 10)

        # Mocap pairing detection
        _qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._paired_sub = self.create_subscription(
            NamedPoseArray, "/poses", self._paired_cb, _qos)
```

Three subscribers with distinct purposes: `/{drone}/pose` provides the EKF state estimate (used for convergence monitoring, position feedback, and safety checks). `/{drone}/status` provides battery voltage and supervisor flags (~1Hz). `/poses` provides mocap data for pairing detection only — the drone's name appearing in the array means the mocap system has detected and identified the marker.

```python
        # --- Publishers ---
        self._fullstate_pub = self.create_publisher(
            FullState, f"/{self._drone}/cmd_full_state", 1)

        # --- Service clients ---
        self._takeoff_cli = self.create_client(
            Takeoff, f"/{self._drone}/takeoff")
        self._land_cli = self.create_client(
            Land, f"/{self._drone}/land")
        self._notify_cli = self.create_client(
            NotifySetpointsStop,
            f"/{self._drone}/notify_setpoints_stop")
        self._emergency_cli = self.create_client(
            Empty, f"/{self._drone}/emergency")
```

One publisher: `cmd_full_state` for position+velocity streaming (same as sim examples). Four service clients: takeoff, land, notify_setpoints_stop (LLC→HLC transition), and emergency (immediate motor cutoff).

```python
        # --- CSV flight log ---
        log_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "logs")
        log_dir = os.path.abspath(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(
            log_dir, f"hover_{self._drone}_{ts}.csv")
        self._log_file = open(self._log_path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            "time_s", "drone", "state", "source",
            "x", "y", "z", "vx", "vy", "vz",
            "yaw_deg", "battery_v",
        ])
        self._log_t0 = None
```

CSV columns record timestamp, drone name, flight state, data source (ekf/vicon/cmd), position, velocity, yaw, and battery voltage. `source` distinguishes the EKF estimate from raw Vicon position from the commanded setpoint — essential for post-flight analysis. `_log_t0` is set on the first log row (not here) so t=0 corresponds to the script start.

```python
        # --- Keyboard thread ---
        self._kb_stop = threading.Event()
        self._key_pressed = None
        self._key_lock = threading.Lock()
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True,
            name="keyboard")
```

The keyboard thread runs as a daemon — it terminates automatically when the main thread exits. `_kb_stop` signals the thread to stop. `_key_pressed` stores the last keypress (protected by `_key_lock`). The thread is started later, after all pre-flight print output is complete (raw terminal mode mangles newlines).

**If the terminal becomes unreadable** (e.g., the script exits (normally, via Ctrl+C, or via a crash) while the terminal is in raw mode): type `reset` and press Enter **blindly** — the terminal will restore itself even if you cannot see what you are typing.

### 6.4: Callback methods

Add these methods inside the `HoverTestNode` class, after `__init__`:

```python
    def _pose_cb(self, msg: PoseStamped):
        self._current_pose = msg
        self._last_pose_time = time.monotonic()
        p = msg.pose.position
        self._log_row("ekf", p.x, p.y, p.z)

    def _status_cb(self, msg: Status):
        self._battery_v = msg.battery_voltage
        self._supervisor_info = msg.supervisor_info

    def _paired_cb(self, msg: NamedPoseArray):
        for pose in msg.poses:
            if pose.name == self._drone:
                self._paired = True
                p = pose.pose.position
                self._log_row("vicon", p.x, p.y, p.z)
                return
```

`_pose_cb` stores the latest EKF pose and logs it to CSV with source `"ekf"`. `_status_cb` updates battery voltage and supervisor info. `_paired_cb` checks whether the configured drone name appears in the `/poses` array — if it does, the mocap system has successfully paired the marker with the drone identity. The raw Vicon position is also logged to CSV with source `"vicon"`.

```python
    def _log_row(self, source, x, y, z,
                 vx=0.0, vy=0.0, vz=0.0, yaw_deg=0.0):
        if self._log_t0 is None:
            self._log_t0 = time.monotonic()
        t = time.monotonic() - self._log_t0
        batt = self._battery_v
        self._log_writer.writerow([
            f"{t:.4f}", self._drone, self._state.name, source,
            f"{x:.5f}", f"{y:.5f}", f"{z:.5f}",
            f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}",
            f"{yaw_deg:.2f}",
            f"{batt:.2f}" if batt is not None else "",
        ])
```

### 6.5: Helper methods

Add inside the `HoverTestNode` class:

```python
    def _get_position(self):
        """Return (x, y, z) from latest EKF pose, or None."""
        if self._current_pose is not None:
            p = self._current_pose.pose.position
            return (p.x, p.y, p.z)
        return None
```

### 6.6: publish_full_state

Add inside the `HoverTestNode` class:

```python
    def _publish_full_state(self, x, y, z,
                            vx=0.0, vy=0.0, vz=0.0,
                            ax=0.0, ay=0.0, az=0.0,
                            yaw=0.0):
        """Publish a FullState setpoint."""
        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        msg.acc.x = float(ax)
        msg.acc.y = float(ay)
        msg.acc.z = float(az)

        self._fullstate_pub.publish(msg)
        self._log_row("cmd", x, y, z, vx, vy, vz,
                      math.degrees(yaw))
```

Same as the sim version from Example 1. The method logs every commanded setpoint to CSV with source `"cmd"`.

### 6.7: Service methods

Add inside the `HoverTestNode` class:

```python
    def _call_takeoff(self, height: float, duration: float):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = height
        req.duration = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9))
        future = self._takeoff_cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)
        self._airborne = True

    def _call_land(self, duration: float):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9))
        future = self._land_cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)

    def _call_notify_stop(self):
        req = NotifySetpointsStop.Request()
        req.group_mask = 0
        req.remain_valid_millisecs = 100
        future = self._notify_cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)

    def _call_emergency(self):
        req = Empty.Request()
        future = self._emergency_cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=2.0)
```

Identical pattern to the sim examples. `_call_emergency` sends an `Empty` service request — no arguments, no response. This immediately cuts power to the motors. Called only as a last resort if the normal `land()` fails during emergency handling.

### 6.8: Safety checks and emergency landing

Add inside the `HoverTestNode` class:

```python
    def _check_safety(self, check_min_height: bool = True) -> bool:
        """Check all safety conditions. Returns False if any fail."""
        if self._emergency_requested:
            return False  # keyboard thread requested emergency
        if not self._airborne or self._emergency_triggered:
            return True

        # Pose timeout
        if self._last_pose_time > 0:
            age = time.monotonic() - self._last_pose_time
            if age > self._pose_timeout:
                self.get_logger().error(
                    f"SAFETY: Pose data stale ({age:.2f}s > "
                    f"{self._pose_timeout}s)")
                return False

        # Position bounds
        pos = self._get_position()
        if pos is not None:
            dist = math.sqrt(pos[0]**2 + pos[1]**2)
            if dist > self._max_dist:
                self.get_logger().error(
                    f"SAFETY: Distance {dist:.2f}m > "
                    f"{self._max_dist}m")
                return False
            if pos[2] > self._max_height:
                self.get_logger().error(
                    f"SAFETY: Height {pos[2]:.2f}m > "
                    f"{self._max_height}m")
                return False
            if check_min_height and pos[2] < self._min_height:
                self.get_logger().error(
                    f"SAFETY: Height {pos[2]:.2f}m < "
                    f"{self._min_height}m")
                return False

        # Supervisor: check for tumbled state during flight
        IS_TUMBLED = 0x20
        if self._supervisor_info & IS_TUMBLED:
            self.get_logger().error(
                "SAFETY: Drone tumbled during flight!")
            return False

        # Battery
        if (self._battery_v is not None
                and self._battery_v < self._batt_critical):
            self.get_logger().error(
                f"SAFETY: Battery critical "
                f"({self._battery_v:.2f}V < {self._batt_critical}V)")
            return False

        return True
```

Five safety checks are performed on every streaming loop iteration:

1. **Pose timeout** — no EKF data for > 0.5s means the mocap or radio link has failed
2. **Position bounds** — horizontal distance from origin, maximum height, minimum height.
3. **Drone tumbled** — `IS_TUMBLED` bit (0x20) in `supervisor_info` indicates the drone has flipped. This triggers immediate motor cutoff rather than controlled landing
4. **Battery critical** — voltage below 3.2V means the battery is near depletion

A fifth trigger exists outside `_check_safety`: **keyboard 'E' or Ctrl+C** — the user manually aborts. The keyboard thread sets `_emergency_requested = True` (a flag only — no rclpy calls from the daemon thread). The main thread polls this flag via `_check_safety` (during streaming phases) and the state-machine loops, and calls `_emergency_land()` on the main thread.

All triggers converge on the same emergency procedure:

```python
    def _emergency_land(self):
        """Emergency landing with fallback chain."""
        if self._emergency_triggered:
            return
        self._emergency_triggered = True
        self._state = FlightState.EMERGENCY
        self.get_logger().warn("EMERGENCY LAND initiated")

        # Hold current position briefly (unless tumbled)
        IS_TUMBLED = 0x20
        if not (self._supervisor_info & IS_TUMBLED):
            pos = self._get_position()
            if pos is not None:
                for _ in range(15):  # ~0.5s at 30Hz
                    self._publish_full_state(
                        pos[0], pos[1], pos[2])
                    time.sleep(self._dt)

        # Fallback chain: notify_stop -> land -> emergency
        try:
            self._call_notify_stop()
        except Exception:
            pass
        try:
            self._call_land(self._land_dur)
        except Exception:
            self.get_logger().error(
                "Land failed, calling emergency stop")
            try:
                self._call_emergency()
            except Exception:
                pass
        self._airborne = False
```

The fallback chain ensures the drone descends by any available means: hold position briefly (unless tumbled, since a flipped drone cannot hold position) → notify_setpoints_stop (LLC→HLC transition) → HLC land (controlled descent) → emergency motor cutoff (last resort). Each step is wrapped in `try/except` so failure at one stage does not prevent the next.

### 6.9: Keyboard thread

Add inside the `HoverTestNode` class:

```python
    def _keyboard_loop(self):
        """Daemon thread: reads keyboard input in raw terminal mode."""
        try:
            import termios
            import tty
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)
            has_tty = True
        except ImportError:
            has_tty = False
        except (OSError, ValueError):
            has_tty = False

        try:
            while not self._kb_stop.is_set():
                if has_tty:
                    ready, _, _ = select.select(
                        [sys.stdin], [], [], 0.3)
                    if ready:
                        ch = sys.stdin.read(1)
                    else:
                        continue
                else:
                    time.sleep(1.0)
                    continue

                if ch in ("e", "\x03"):  # \x03 = Ctrl+C
                    self.get_logger().warn(
                        "Emergency key pressed!")
                    self._emergency_requested = True
                    if ch == "\x03":
                        self._kb_stop.set()
                    break
                else:
                    with self._key_lock:
                        self._key_pressed = ch
        finally:
            if has_tty:
                import termios
                termios.tcsetattr(
                    fd, termios.TCSADRAIN, old_settings)
```

`termios`/`tty` are Unix-specific. `tty.setraw(fd)` puts the terminal in raw mode — each keypress is available immediately without waiting for Enter. The `select.select` call waits up to 0.3 seconds for input, then times out so the loop can check `_kb_stop`. The `has_tty` fallback handles environments where terminal control is unavailable (IDEs, some terminals, Windows) — in that case the thread sleeps for 1 second between checks and keyboard input will not work, but the script will not crash.

```python
    def _consume_key(self) -> str | None:
        """Read and clear the last stored keypress."""
        with self._key_lock:
            k = self._key_pressed
            self._key_pressed = None
            return k
```

### 6.10: Terminal-aware output helpers

Add these methods inside the `HoverTestNode` class, before `run()`. The keyboard thread puts the terminal into raw mode (no line buffering), so normal `print()` with `\n` alone causes stair-stepped output. `_print` emits `\r\n` to compensate.

```python
    def _print(self, text=""):
        """Print with \\r\\n so output is correct in raw terminal mode."""
        for line in text.split("\n"):
            sys.stdout.write(line + "\r\n")
        sys.stdout.flush()

    def _print_header(self):
        self._print("\n" + "=" * 55)
        self._print("  HOVER TEST (Crazyswarm2 + cflib)")
        self._print("=" * 55)
        self._print(f"  Drone:       {self._drone}")
        self._print(f"  Height:      {self._height * 100:.0f} cm")
        self._print(f"  Hover time:  {self._hover_dur} s")
        self._print(f"  Safety:      max_dist={self._max_dist}m, "
                     f"pose_timeout={self._pose_timeout}s")
        self._print("=" * 55)
        self._print("  Keys: [T] takeoff | [H] hover | [E] emergency")
        self._print("=" * 55)
```

### 6.11: The main run() state machine

Add the `run` method inside the `HoverTestNode` class. This is the core of the script — a sequential state machine where each state blocks until its exit condition is met (keypress, timeout, or data condition).

```python
    def run(self):
        rate = RateController(self._dt)
        self._print_header()

        # Wait for services
        self.get_logger().info(
            "Waiting for crazyflie_server services...")
        if not self._takeoff_cli.wait_for_service(
                timeout_sec=10.0):
            self.get_logger().error(
                "Takeoff service not available! "
                "Is crazyflie_server running?")
            return
        self.get_logger().info("Services ready.")
```

#### Pre-flight states

**WAITING_FOR_PAIR** — blocks until the drone's name appears in `/poses`:

```python
        self._print(f"\n  Waiting for drone '{self._drone}' "
                    f"in /poses (motion capture)...")
        while not self._paired and not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._kb_stop.is_set():
            return
```

**WAITING_FOR_EKF** — blocks until EKF pose data is available and has converged:

```python
        self._print(f"\n  Drone '{self._drone}' paired! "
                    f"Waiting for EKF pose data...")
        start = time.monotonic()
        while (self._current_pose is None
               and time.monotonic() - start < 10.0):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._current_pose is None:
            self.get_logger().error(
                "No pose data received after pairing!")
            return

        # Wait for EKF convergence (position spread < 1cm)
        self._print("  Waiting for EKF to converge...")
        converge_start = time.monotonic()
        samples = []
        while time.monotonic() - converge_start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            pos = self._get_position()
            if pos is not None:
                samples.append(pos)
                if len(samples) >= 20:
                    recent = samples[-20:]
                    xs = [p[0] for p in recent]
                    ys = [p[1] for p in recent]
                    zs = [p[2] for p in recent]
                    spread = max(
                        max(xs) - min(xs),
                        max(ys) - min(ys),
                        max(zs) - min(zs))
                    if spread < 0.01:  # converged
                        break
        else:
            self.get_logger().warn(
                "EKF convergence timeout — proceeding anyway")
```

The convergence check collects 20+ position samples from the EKF. If the max-minus-min spread on all three axes is under 1cm, the estimate is stable. If convergence times out after 15 seconds, the script warns but proceeds — the user can abort with 'E'.

**Why the Kalman filter is NOT reset:** Single-marker Vicon provides position only (NaN quaternion = no orientation). If the Kalman filter were reset, the yaw estimate would zero out with no external yaw measurement available to re-converge it. The PID position controller works in a body-yaw-aligned frame — with a wrong yaw estimate, corrections are applied in a rotated frame, degrading position-hold authority.

**WAITING_FOR_CAN_FLY** — blocks until sensor self-test passes:

```python
        pos = self._get_position()
        self._print(f"  EKF converged. Position: "
                    f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

        CAN_FLY = 0x08
        IS_TUMBLED = 0x20
        self._print(f"  Waiting for sensor calibration "
                    f"(supervisor CAN_FLY)...")
        sup_start = time.monotonic()
        while time.monotonic() - sup_start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._supervisor_info & IS_TUMBLED:
                self.get_logger().error(
                    "Drone is tumbled! Place it upright "
                    "and restart.")
                return
            if self._supervisor_info & CAN_FLY:
                break
        else:
            self.get_logger().warn(
                "Supervisor CAN_FLY not set after 15s — "
                "proceeding, but sensors may not be ready")

        batt_str = (f"{self._battery_v:.2f}V"
                    if self._battery_v is not None
                    else "unknown")
        self._print(f"  Sensors ready. Battery: {batt_str}")
```

#### Flight states

**Starting the keyboard thread** — must happen after all print output. Raw terminal mode mangles newlines, so the thread starts after the pre-flight status is displayed:

```python
        self._kb_thread.start()
```

**READY_FOR_TAKEOFF** — waits for 'T' key:

```python
        self._print(f"\n  >>> Press 'T' to take off to "
                    f"{self._height*100:.0f}cm <<<")
        self._print(f"  >>> Press 'E' at any time for "
                    f"emergency land <<<\n")

        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._emergency_requested:
                self._emergency_land()
                return
            if self._emergency_triggered:
                return
            key = self._consume_key()
            if key == "t":
                break
        if self._kb_stop.is_set():
            return
```

**TAKING_OFF** — calls the HLC takeoff service, verifies height reached:

```python
        self._state = FlightState.TAKING_OFF
        self._print(f"\n  [TAKEOFF] Taking off to "
                    f"{self._height}m...")
        self._call_takeoff(self._height, self._takeoff_dur)

        takeoff_wait = self._takeoff_dur + 1.0
        wait_start = time.monotonic()
        while time.monotonic() - wait_start < takeoff_wait:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._check_safety(check_min_height=False):
                self._emergency_land()
                return

        # Verify takeoff succeeded
        pos = self._get_position()
        if pos is not None:
            self._print(f"  [TAKEOFF] Height after takeoff: "
                        f"{pos[2]*100:.1f}cm")
            if pos[2] < self._height * 0.6:
                self.get_logger().error(
                    f"Takeoff failed! "
                    f"Height={pos[2]:.3f}m "
                    f"(need >{self._height*0.6:.2f}m)")
                self._emergency_land()
                return
```

The drone must reach at least 60% of the target height. If not, something failed (dead battery, motor issue, EKF divergence). The script exits without attempting further flight.

**STABILIZING** — lets the HLC hold position after takeoff:

```python
        self._state = FlightState.STABILIZING
        self._print(f"  [STABILIZE] HLC holding position for "
                    f"{self._hover_stab}s...")
        stab_start = time.monotonic()
        while (time.monotonic() - stab_start
               < self._hover_stab):
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._check_safety(check_min_height=False):
                self._emergency_land()
                return
```

During stabilizing, **no LLC commands are sent.** The HLC takeoff planner naturally holds at the target height after completing the takeoff. Sending LLC commands would fight the HLC and cause instability.

> [!WARNING]
> **Yaw alignment:** Single-marker Vicon provides position only (no orientation — the quaternion in `/poses` is NaN). Yaw is unobservable and cannot be corrected automatically. Before takeoff, physically place the drone on the floor facing the world +X direction (use the Vicon coordinate system axes for reference). The EKF maintains whatever yaw estimate it starts with, and the PID controller works in a body-yaw-aligned frame — so the drone must start facing +X to match the world coordinate system used by all setpoint commands.

**READY_TO_HOVER** — waits for 'H' key while hovering at current position. The drone streams `cmd_full_state` at its current position with `yaw=0` (world +X aligned):

```python
        # READY_TO_HOVER — wait for 'H'
        self._state = FlightState.READY_TO_HOVER
        self._print(f"\n  >>> Press 'H' to hover for "
                    f"{self._hover_dur}s <<<\n")

        rate.start()
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.001)
            cur = self._get_position()
            if cur is not None:
                self._publish_full_state(
                    cur[0], cur[1], self._height)
            if not self._check_safety():
                self._emergency_land()
                return
            if self._consume_key() == "h":
                break
            rate.sleep()
        if self._kb_stop.is_set():
            self._emergency_land()
            return
```

**HOVERING** — streams `cmd_full_state` at the current position for the configured duration:

```python
        self._state = FlightState.HOVERING
        self._print(f"  [HOVER] Hovering for "
                    f"{self._hover_dur}s...")
        hover_start = time.monotonic()
        rate.start()
        while (time.monotonic() - hover_start
               < self._hover_dur):
            rclpy.spin_once(self, timeout_sec=0.001)
            cur = self._get_position()
            if cur is not None:
                self._publish_full_state(
                    cur[0], cur[1], self._height)
            if not self._check_safety():
                self._emergency_land()
                return
            rate.sleep()

        if rate.overruns:
            self.get_logger().warning(
                f'{rate.overruns} loop overruns at '
                f'{self._rate_hz}Hz')
```

Safety is checked on every iteration of the hover loop. If pose data goes stale, the drone drifts outside bounds, the supervisor reports a tumble, or the battery drops, emergency landing triggers immediately.

**LANDING** — transitions from LLC back to HLC, then lands:

```python
        self._state = FlightState.LANDING
        self._print(f"\n  Landing...")
        self._call_notify_stop()
        time.sleep(0.1)
        self._call_land(self._land_dur)
        time.sleep(self._land_dur + 1.0)
        self._airborne = False
```

`notify_setpoints_stop()` tells the firmware to stop listening for streaming setpoints and switch back to HLC mode. The `time.sleep(0.1)` gives the firmware a moment to complete the mode switch. `land()` calls the HLC land service — the drone descends to 0.0m over `land_dur` seconds.

**DONE** — prints the flight summary:

```python
        self._state = FlightState.DONE
        self._print(f"\n  Flight complete!")
        pos = self._get_position()
        if pos is not None:
            self._print(f"  Final position: "
                        f"({pos[0]:.3f}, {pos[1]:.3f}, "
                        f"{pos[2]:.3f})")
        if self._battery_v is not None:
            self._print(f"  Battery: {self._battery_v:.2f}V")
        self._print(f"  Log: {self._log_path}")
```

The summary displays the drone's final position after landing, the battery voltage, and the path to the CSV flight log.

### 6.11: CSV log cleanup and shutdown

Add inside the `HoverTestNode` class:

```python
    def _close_log(self):
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    def shutdown(self):
        self._kb_stop.set()
        if self._airborne and not self._emergency_triggered:
            self.get_logger().warn(
                "Shutting down while airborne — "
                "emergency landing!")
            self._emergency_land()
        self._close_log()
```

### 6.12: main function

Add at the bottom of the file:

```python
def main():
    rclpy.init()
    node = HoverTestNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(
            f"Unhandled exception: {e}")
        if node._airborne:
            node._emergency_land()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

`KeyboardInterrupt` is caught silently — the keyboard thread already triggered emergency landing on Ctrl+C. Any other exception triggers emergency landing (if airborne) before shutdown.

---

## Section 7: Register Entry Points

Edit `setup.py`. Open `hardware_hover/setup.py` and replace its contents:

```python
from setuptools import setup
from glob import glob

package_name = 'hardware_hover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='your-email',
    description='Example 5: Hardware hover test',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hover_test = hardware_hover.hover_test:main',
        ],
    },
)
```

---

## Pre-flight safety checklist

Before any launch command, verify every item:

- [ ] Propellers are installed correctly and not damaged
- [ ] Drone is placed on a flat surface facing world **+X**
- [ ] Vicon marker is visible in `/poses` and paired to the correct drone name
- [ ] EKF has converged (position spread < 1 cm)
- [ ] Supervisor `CAN_FLY` bit is set (0x08)
- [ ] Battery voltage is above 3.2 V
- [ ] Emergency key ('E') is understood — press it at any time for immediate landing
- [ ] Flight area is clear of people and obstacles

> [!WARNING]
> **Yaw orientation matters.** Single-marker Vicon provides position only — no orientation. The EKF maintains whatever yaw estimate it starts with, and the PID controller works in a body-yaw-aligned frame. The drone **must** face world +X before takeoff or all position corrections will be applied in a rotated frame.

---

## Section 8: Build and Test

### 8.1: Build

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
colcon build --symlink-install
```

### 8.2: Manually verify mocap and EKF data (CRITICAL)

> [!CAUTION]
> **Never skip this step.** Before running the flight script, the data pipeline must be verified manually using ROS2 command-line tools.

**Step 1: Launch the CS2 server.** Terminal 1:
```bash
source install/setup.bash
ros2 launch hardware_hover hardware_hover.launch.py
```

**Step 2: Verify /poses.** Terminal 2:
```bash
ros2 topic echo /poses
```

Look for an entry with `name: "cf1"`. The position should match the physical marker placement. If no entry appears, the mocap does not see the drone — check marker visibility, Vicon Tracker streaming state, and `motion_capture.yaml`. The orientation fields should show `.nan` — correct for single-marker tracking.

**Step 3: Verify /cf1/pose.** Terminal 2:
```bash
ros2 topic echo /cf1/pose
```

Compare the EKF position against the Vicon position from Step 2. After EKF convergence (wait 15-30 seconds), the values should be within ~5cm in XY and ~2cm in Z. Large discrepancies indicate the EKF is not correctly fusing external position data.

**Step 4: Verify /cf1/status.** Terminal 2:
```bash
ros2 topic echo /cf1/status
```

Check `battery_voltage` (> 3.8V for a charged battery, < 3.2V is critical). Check `supervisor_info` — the CAN_FLY bit (0x08) should be set. If it is 0, place the drone on a flat, level surface and wait for sensors to self-calibrate.

**Step 5: Visual verification with the viewer (recommended).** Terminal 3:
```bash
ros2 run vicon_viewer drone_viewer
```

The viewer built in Example 4 displays real-time drone positions from `/poses`. Confirm the marker appears at the expected physical position with correct trails and labels. This provides a visual sanity check before flight.

**If all checks pass:** The data pipeline is working. Kill `ros2 topic echo` with Ctrl+C. Keep the CS2 server running in Terminal 1. The viewer in Terminal 3 can remain open during flight to monitor the drone in real time.

### 8.3: Run the flight script

Terminal 2 (or a new terminal if the viewer is using Terminal 2):
```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run hardware_hover hover_test --ros-args -p drone_name:=cf1
```

Alternatively, launch everything together (CS2 server + viewer) in Terminal 1:
```bash
source install/setup.bash
ros2 launch hardware_hover hardware_hover.launch.py viewer:=true
```
Then run the flight script in Terminal 2.

**Expected flow:**
1. Script prints header with flight parameters and safety bounds
2. "Waiting for drone 'cf1' in /poses..." → drone found
3. "Waiting for EKF to converge..." → position stabilizes (spread < 1cm)
4. "Waiting for sensor calibration (supervisor CAN_FLY)..." → self-test passes
5. Battery voltage displayed
6. "Press 'T' to take off to 30cm" → press T → drone takes off
7. "Press 'H' to hover" → press H → drone hovers 5 seconds
8. Auto-lands, prints summary (final position, battery)

### 8.4: Post-flight

Check the CSV log in `hardware_hover/logs/`:
- Compare `source=ekf` vs `source=vicon` positions — should track closely
- Check `source=cmd` to see commanded setpoints
- Verify battery voltage remained stable during flight

---

## Potential Issues

- **Script hangs on "Waiting for drone in /poses"** — the mocap system does not see the drone. Check Vicon Tracker is streaming, the marker is visible, and the drone name matches `crazyflies.yaml`.
- **EKF never converges (spread > 1cm for > 15s)** — the drone may be moving (vibrations) or the marker may be occluded. Place on a stable surface.
- **Takeoff fails (height < 60% of target)** — low battery, damaged propellers, or motor issue.
- **Drone drifts during hover** — ensure the drone was physically placed facing world +X before takeoff. Single-marker Vicon cannot correct yaw alignment. If drift persists, power cycle the drone and re-check physical alignment.
- **Emergency triggers immediately** — safety bounds may be too tight. Check `max_distance_m` relative to the drone's placement.
- **'E' key does not work** — the terminal may not support raw mode. Run in a standard terminal (gnome-terminal, konsole, xterm).
- **Terminal is garbled / can't see what you're typing** — the script may have exited (or crashed) while the terminal was in raw mode. Type `reset` and press Enter blindly to restore the terminal.
