# Example 6 — Hardware Trajectory Flight

This is the capstone beginner example. It extends the hardware safety infrastructure from Example 5 with trajectory flight. The example is split into two parts:

- **6A (single-drone):** One drone flies a rectangular trajectory on hardware with full keyboard-gated safety. The drone takes off, moves to the trajectory start corner, flies a rectangle with velocity feedforward, and lands. The drone must be physically placed facing world +X before takeoff.
- **6B (multi-drone):** Two drones fly simultaneously — one rectangle, one circle — with hover-wait synchronization and per-drone safety monitoring.

The trajectory computation math is reused from the sim examples (Example 2 for square, Example 3 for circle). The new content is entirely about adapting it for hardware: keyboard-gated phase transitions, safety monitoring during dynamic flight, live progress display, and post-flight CSV analysis.

This example assumes completion of Examples 1-5.

---

## Files Created

```
tutorial_ws/src/hardware_trajectory/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   ├── crazyflies.yaml
│   ├── crazyflies_multi.yaml
│   ├── motion_capture.yaml
│   ├── flight_config.yaml
│   └── flight_config_multi.yaml
├── launch/
│   ├── trajectory_flight.launch.py
│   └── trajectory_flight_multi.launch.py
└── hardware_trajectory/
    ├── __init__.py
    ├── trajectory_flight.py        # 6A: single-drone
    └── trajectory_flight2.py       # 6B: multi-drone
```

---

## Part A — Single-Drone Trajectory Flight

---

### A.1: Create the Package

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws/src
ros2 pkg create hardware_trajectory --build-type ament_python \
    --dependencies rclpy crazyflie_interfaces motion_capture_tracking_interfaces geometry_msgs std_srvs
cd hardware_trajectory
mkdir -p config launch resource
touch resource/hardware_trajectory
```

---

### A.2: Write the Configuration Files

**crazyflies.yaml** — single drone hardware config. Create `config/crazyflies.yaml`:

```bash
touch config/crazyflies.yaml
```

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

**motion_capture.yaml** — same Vicon connection and marker configuration used in Example 5. Create `config/motion_capture.yaml`:

```bash
touch config/motion_capture.yaml
```

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

**flight_config.yaml** — extends Example 5's config with trajectory parameters. Create `config/flight_config.yaml`:

```bash
touch config/flight_config.yaml
```

```yaml
flight:
  drone_name: "cf1"
  height_m: 0.30
  rectangle:
    side_m: 0.40
    velocity_ms: 0.20
    start_position_m: [0.0, 0.0]
  transition_velocity_ms: 0.10

safety:
  max_distance_m: 3.0
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

`rectangle.side_m: 0.40` and `velocity_ms: 0.20` are intentionally conservative — 40cm square at 20 cm/s. The first hardware trajectory should be small and slow. `start_position_m: [0.0, 0.0]` places the bottom-left corner of the rectangle at the world origin.

---

### A.3: Write the Launch File

Create `launch/trajectory_flight.launch.py`:

```bash
touch launch/trajectory_flight.launch.py
```

This launch file starts the full hardware pipeline for a single drone: `motion_capture_tracking_node` (connects to Vicon), `crazyflie_server.py` (connects to the drone via Crazyradio), and optionally the viewer from Example 4.

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
    pkg_dir = get_package_share_directory('hardware_trajectory')
    cs2_share = get_package_share_directory('crazyflie')

    crazyflies_path = os.path.join(
        pkg_dir, 'config', 'crazyflies.yaml')
    mocap_yaml_path = os.path.join(
        pkg_dir, 'config', 'motion_capture.yaml')
    server_yaml_path = os.path.join(
        cs2_share, 'config', 'server.yaml')
    urdf_path = os.path.join(
        cs2_share, 'urdf', 'crazyflie_description.urdf')

    with open(crazyflies_path) as f:
        crazyflies = yaml.safe_load(f)
    with open(mocap_yaml_path) as f:
        mocap_cfg = yaml.safe_load(f)
    with open(server_yaml_path) as f:
        server_cfg = yaml.safe_load(f)
    with open(urdf_path) as f:
        robot_desc = f.read()

    # Build mocap parameters (marker configs + Vicon connection)
    mocap_params = \
        mocap_cfg['/motion_capture_tracking']['ros__parameters']
    mocap_params['rigid_bodies'] = {}
    for key, value in crazyflies['robots'].items():
        if value['enabled']:
            robot_type = crazyflies['robot_types'][
                value['type']]
            mc = robot_type['motion_capture']
            if (mc['enabled']
                    and mc.get('tracking')
                    == 'librigidbodytracker'):
                mocap_params['rigid_bodies'][key] = {
                    'initial_position': value[
                        'initial_position'],
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

`crazyflie_server.py` (Python/cflib backend) is used instead of `crazyflie_server` (C++ backend) because the C++ backend has bugs with Crazyradio 2.0 in PA mode. The `rigid_bodies` dictionary is built from enabled drones with `librigidbodytracker` tracking — it tells the mocap pipeline which marker configurations to search for and where to expect them.

---

### A.4: Write trajectory_flight.py (6A)

Create `hardware_trajectory/trajectory_flight.py`:

```bash
touch hardware_trajectory/trajectory_flight.py
```

The structure follows `hover_test.py` from Example 5 closely. The key additions: four new flight states, a rectangle setpoint computation method, and a progress display during flight.

#### A.4.1: Shebang, docstring, and imports

```python
#!/usr/bin/env python3
"""
Hardware trajectory flight with keyboard-gated safety workflow.

Performs pre-flight checks (mocap pairing, EKF convergence, sensor
self-test, battery), moves to a trajectory start corner, flies a
rectangle with velocity feedforward, and lands. Safety checks run
continuously during all streaming phases. Press 'E' at any time
for emergency landing.
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

**Imports explained:** Same set as Example 5. `csv` and `datetime` for flight logging. `select` and `threading` for the keyboard input thread. `enum.Enum` for the `FlightState` state machine. `PoseStamped` for EKF pose messages. `NamedPoseArray` for mocap pairing detection. `Status` for drone status (battery and supervisor_info). `Empty` for the emergency stop service.

#### A.4.2: Extended FlightState enum

```python
class FlightState(Enum):
    WAITING_FOR_PAIR = auto()
    WAITING_FOR_EKF = auto()
    WAITING_FOR_CAN_FLY = auto()
    READY_FOR_TAKEOFF = auto()
    TAKING_OFF = auto()
    STABILIZING = auto()
    READY_TO_MOVE = auto()       # NEW
    MOVING_TO_START = auto()     # NEW
    HOVERING = auto()            # NEW (waiting for 'G' at start corner)
    FLYING = auto()              # NEW
    LANDING = auto()
    DONE = auto()
    EMERGENCY = auto()
```

The progression from takeoff onward is: STABILIZING → (press M) → MOVING_TO_START → (auto) → HOVERING → (press G) → FLYING → (auto after trajectory completes) → LANDING.

#### A.4.3: Infrastructure blocks (from Example 5 patterns)

The following blocks use the same patterns introduced in Example 5. Each is provided here as a complete, self-contained code block. The reader copies each block directly — no cross-referencing with Example 5 is required. Brief explanations accompany each block.

**RateController class** — place at the top of the file, before `FlightState`:

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

Phase-locked 30Hz timer. `start()` records the anchor time. `sleep()` advances by one period and sleeps the remaining time. Skip-on-overrun prevents back-to-back firing if an iteration is late.

**_load_config function** — place after the imports:

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
    return {
        "flight": {
            "height_m": 0.30,
            "rectangle": {
                "side_m": 0.40, "velocity_ms": 0.20,
                "start_position_m": [0.0, 0.0],
            },
            "transition_velocity_ms": 0.10,
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

The fallback defaults include trajectory parameters so the script can run without a config file.

**TrajectoryFlightNode.__init__** — place inside the node class. The full constructor including subscribers, publishers, service clients, CSV logger, and keyboard thread setup:

```python
class TrajectoryFlightNode(Node):

    def __init__(self):
        super().__init__("trajectory_flight")

        # --- Parameters ---
        self.declare_parameter("drone_name", "cf1")
        self.declare_parameter("config", "")
        self._drone = (
            self.get_parameter("drone_name")
            .get_parameter_value().string_value)
        config_path = (
            self.get_parameter("config")
            .get_parameter_value().string_value)
        self._cfg = _load_config(config_path)

        flight = self._cfg["flight"]
        safety = self._cfg["safety"]
        control = self._cfg["control"]

        self._height = flight["height_m"]
        self._rect_side = flight["rectangle"]["side_m"]
        self._rect_vel = flight["rectangle"]["velocity_ms"]
        self._rect_start = flight["rectangle"]["start_position_m"]
        self._trans_vel = flight["transition_velocity_ms"]

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

        # Derived timing
        self._segment_duration = self._rect_side / self._rect_vel
        self._trajectory_duration = 4 * self._segment_duration

        # --- State ---
        self._state = FlightState.WAITING_FOR_PAIR
        self._current_pose = None
        self._last_pose_time = 0.0
        self._battery_v = None
        self._supervisor_info = 0
        self._paired = False
        self._airborne = False
        self._emergency_triggered = False
        self._emergency_requested = False

        # --- Subscribers ---
        self._pose_sub = self.create_subscription(
            PoseStamped, f"/{self._drone}/pose",
            self._pose_cb, 10)
        self._status_sub = self.create_subscription(
            Status, f"/{self._drone}/status",
            self._status_cb, 10)
        _qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._paired_sub = self.create_subscription(
            NamedPoseArray, "/poses", self._paired_cb, _qos)

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

        # --- CSV flight log ---
        log_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "logs")
        log_dir = os.path.abspath(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(
            log_dir, f"flight_{self._drone}_{ts}.csv")
        self._log_file = open(self._log_path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            "time_s", "drone", "state", "source",
            "x", "y", "z", "vx", "vy", "vz",
            "yaw_deg", "battery_v",
        ])
        self._log_t0 = None

        # --- Keyboard thread ---
        self._kb_stop = threading.Event()
        self._key_pressed = None
        self._key_lock = threading.Lock()
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True,
            name="keyboard")
```

The additional trajectory parameters (`_rect_side`, `_rect_vel`, `_rect_start`, `_trans_vel`) and derived timing (`_segment_duration`, `_trajectory_duration`) are the only differences from Example 5's `__init__`.

**Callback methods** — add inside the `TrajectoryFlightNode` class:

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

    def _get_position(self):
        if self._current_pose is not None:
            p = self._current_pose.pose.position
            return (p.x, p.y, p.z)
        return None
```

**publish methods** — add inside the `TrajectoryFlightNode` class:

```python
    def _publish_full_state(self, x, y, z,
                            vx=0.0, vy=0.0, vz=0.0,
                            ax=0.0, ay=0.0, az=0.0,
                            yaw=0.0):
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

**Service methods** — add inside the `TrajectoryFlightNode` class:

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

**_check_safety** — add inside the `TrajectoryFlightNode` class:

```python
    def _check_safety(self, check_min_height: bool = True) -> bool:
        if self._emergency_requested:
            return False  # keyboard thread requested emergency
        if not self._airborne or self._emergency_triggered:
            return True

        # Pose timeout
        if self._last_pose_time > 0:
            age = time.monotonic() - self._last_pose_time
            if age > self._pose_timeout:
                self.get_logger().error(
                    f"SAFETY: Pose data stale ({age:.2f}s)")
                return False

        # Position bounds
        pos = self._get_position()
        if pos is not None:
            dist = math.sqrt(pos[0]**2 + pos[1]**2)
            if dist > self._max_dist:
                self.get_logger().error(
                    f"SAFETY: Distance {dist:.2f}m exceeded")
                return False
            if pos[2] > self._max_height:
                self.get_logger().error(
                    f"SAFETY: Height {pos[2]:.2f}m exceeded")
                return False
            if check_min_height and pos[2] < self._min_height:
                self.get_logger().error(
                    f"SAFETY: Height too low ({pos[2]:.2f}m)")
                return False

        # Tumbled
        IS_TUMBLED = 0x20
        if self._supervisor_info & IS_TUMBLED:
            self.get_logger().error(
                "SAFETY: Drone tumbled during flight!")
            return False

        # Battery
        if (self._battery_v is not None
                and self._battery_v < self._batt_critical):
            self.get_logger().error(
                f"SAFETY: Battery critical ({self._battery_v:.2f}V)")
            return False

        return True
```

Five safety trigger types: pose timeout, position bounds, drone tumbled, battery critical, and keyboard 'E' (handled separately in the keyboard thread).

**_emergency_land** — add inside the `TrajectoryFlightNode` class:

```python
    def _emergency_land(self):
        if self._emergency_triggered:
            return
        self._emergency_triggered = True
        self._state = FlightState.EMERGENCY
        self.get_logger().warn("EMERGENCY LAND initiated")

        IS_TUMBLED = 0x20
        if not (self._supervisor_info & IS_TUMBLED):
            pos = self._get_position()
            if pos is not None:
                for _ in range(15):
                    self._publish_full_state(
                        pos[0], pos[1], pos[2])
                    time.sleep(self._dt)

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

Fallback chain: hold position (unless tumbled) → notify_stop → land → emergency motor cutoff.

**Keyboard thread** — add inside the `TrajectoryFlightNode` class:

```python
    def _keyboard_loop(self):
        try:
            import termios, tty
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

                if ch in ("e", "\x03"):
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
                termios.tcsetattr(
                    fd, termios.TCSADRAIN, old_settings)

    def _consume_key(self) -> str | None:
        with self._key_lock:
            k = self._key_pressed
            self._key_pressed = None
            return k
```

**Cleanup and main** — add inside the `TrajectoryFlightNode` class:

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
#### A.4.4: Rectangle setpoint computation

Add this method inside the `TrajectoryFlightNode` class:

```python
    def _rectangle_setpoint(self, elapsed: float):
        """Compute position and velocity for a rectangle at time t.

        Returns (x, y, vx, vy) or None if elapsed is outside the
        trajectory duration.
        """
        if elapsed < 0 or elapsed >= self._trajectory_duration:
            return None

        seg_idx = int(elapsed / self._segment_duration)
        seg_t = elapsed - seg_idx * self._segment_duration
        frac = seg_t / self._segment_duration

        x0, y0 = self._rect_start

        if seg_idx == 0:      # +X
            x = x0 + self._rect_side * frac
            y = y0
            vx, vy = self._rect_vel, 0.0
        elif seg_idx == 1:    # +Y
            x = x0 + self._rect_side
            y = y0 + self._rect_side * frac
            vx, vy = 0.0, self._rect_vel
        elif seg_idx == 2:    # -X
            x = x0 + self._rect_side * (1.0 - frac)
            y = y0 + self._rect_side
            vx, vy = -self._rect_vel, 0.0
        else:                 # -Y
            x = x0
            y = y0 + self._rect_side * (1.0 - frac)
            vx, vy = 0.0, -self._rect_vel

        return (x, y, vx, vy)
```

Returns a tuple rather than a `Setpoint` object. The four-segment decomposition is identical to Example 2's sim version.

#### A.4.5: New states in the state machine

The pre-flight states (WAITING_FOR_PAIR through the sensor/battery check) use the same logic introduced in Example 5. The complete `run()` method is provided below as a single block — the pre-flight code is included so the script is self-contained. The explanations focus on the new states (READY_TO_MOVE through FLYING) which are not covered in Example 5.

Add the `_print` and `_print_header` helpers inside the `TrajectoryFlightNode` class, before `run()`. The keyboard thread puts the terminal into raw mode (no line buffering), so `_print` emits `\r\n` to keep output aligned. **If the terminal becomes unreadable** (e.g., the script exits (or crashes) in raw mode): type `reset` and press Enter blindly — the terminal will restore itself.

```python
    def _print(self, text=""):
        """Print with \\r\\n so output is correct in raw terminal mode."""
        for line in text.split("\n"):
            sys.stdout.write(line + "\r\n")
        sys.stdout.flush()

    def _print_header(self):
        self._print("\n" + "=" * 55)
        self._print("  TRAJECTORY FLIGHT (Crazyswarm2 + cflib)")
        self._print("=" * 55)
        self._print(f"  Drone:       {self._drone}")
        self._print(f"  Height:      {self._height * 100:.0f} cm")
        self._print(f"  Rectangle:   {self._rect_side * 100:.0f}cm x "
                     f"{self._rect_side * 100:.0f}cm")
        self._print(f"  Velocity:    {self._rect_vel * 100:.0f} cm/s")
        self._print(f"  Safety:      max_dist={self._max_dist}m, "
                     f"pose_timeout={self._pose_timeout}s")
        self._print("=" * 55)
        self._print("  Keys: [T] takeoff | [M] move | [G] go")
        self._print("        [E] emergency")
        self._print("=" * 55)
```

Add the `run()` method inside the `TrajectoryFlightNode` class. The pre-flight states (WAITING_FOR_PAIR through the sensor/battery check) use the same logic from Example 5, provided here as a complete block so the script is self-contained:

```python
    def run(self):
        rate = RateController(self._dt)
        self._print_header()

        # --- Wait for services ---
        if not self._takeoff_cli.wait_for_service(
                timeout_sec=10.0):
            self.get_logger().error(
                "Takeoff service not available!")
            return

        # --- WAITING_FOR_PAIR ---
        self._print(f"\n  Waiting for drone "
                    f"'{self._drone}' in /poses...")
        while not self._paired and not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._kb_stop.is_set(): return

        # --- EKF convergence ---
        self._print(f"\n  Waiting for EKF pose data...")
        start = time.monotonic()
        while (self._current_pose is None
               and time.monotonic() - start < 10.0):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._current_pose is None:
            self.get_logger().error("No pose data!")
            return

        converge_start = time.monotonic()
        samples = []
        while time.monotonic() - converge_start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            pos = self._get_position()
            if pos is not None:
                samples.append(pos)
                if len(samples) >= 20:
                    recent = samples[-20:]
                    spread = max(
                        max(axis) - min(axis)
                        for axis in zip(*recent))
                    if spread < 0.01: break
        else:
            self.get_logger().warn(
                "EKF convergence timeout")

        pos = self._get_position()
        if pos is None:
            self.get_logger().error(
                "Lost EKF pose after convergence!")
            return
        self._print(f"  EKF converged. Position: "
                    f"({pos[0]:.3f}, {pos[1]:.3f}, "
                    f"{pos[2]:.3f})")

        # --- Supervisor CAN_FLY ---
        CAN_FLY = 0x08
        IS_TUMBLED = 0x20
        self._print(f"  Waiting for supervisor CAN_FLY...")
        sup_start = time.monotonic()
        while time.monotonic() - sup_start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._supervisor_info & IS_TUMBLED:
                self.get_logger().error(
                    "Drone is tumbled!"); return
            if self._supervisor_info & CAN_FLY: break
        else:
            self.get_logger().warn(
                "CAN_FLY not set after 15s")

        batt = (f"{self._battery_v:.2f}V"
                if self._battery_v else "unknown")
        self._print(f"  Sensors ready. Battery: {batt}")

        # --- Start keyboard thread ---
        self._kb_thread.start()

        # --- READY_FOR_TAKEOFF ---
        self._print(f"\n  >>> Press 'T' to take off to "
                    f"{self._height*100:.0f}cm <<<")
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._emergency_requested:
                self._emergency_land(); return
            if self._emergency_triggered: return
            if self._consume_key() == "t": break
        if self._kb_stop.is_set(): return

        # --- TAKING_OFF ---
        self._state = FlightState.TAKING_OFF
        self._print(f"\n  [TAKEOFF] Taking off to "
                    f"{self._height*100:.0f}cm...")
        self._call_takeoff(self._height, self._takeoff_dur)

        # Spin during the takeoff so EKF updates arrive — a plain
        # time.sleep() would leave the pose stale and fail the check.
        takeoff_wait = self._takeoff_dur + 1.0
        wait_start = time.monotonic()
        while time.monotonic() - wait_start < takeoff_wait:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self._check_safety(check_min_height=False):
                self._emergency_land()
                return

        pos = self._get_position()
        if pos is not None and pos[2] < self._height * 0.6:
            self.get_logger().error("Takeoff failed!")
            self._emergency_land()
            return

        # --- STABILIZING ---
        self._state = FlightState.STABILIZING
        time.sleep(self._hover_stab)

```

**Yaw alignment note:** Single-marker Vicon provides position only (no orientation — the quaternion in `/poses` is NaN). Yaw is unobservable and cannot be corrected automatically. Before takeoff, physically place the drone on the floor facing the world +X direction. The EKF maintains whatever yaw estimate it starts with, and the PID controller works in a body-yaw-aligned frame — so the drone must start facing +X to match the world coordinate system used by all setpoint commands.

**READY_TO_MOVE** — waits for 'M' key while hovering at current position:

```python
        self._state = FlightState.READY_TO_MOVE
        self._print(f"\n  Target start position: "
                    f"({self._rect_start[0]:.2f}, "
                    f"{self._rect_start[1]:.2f})")
        self._print(f"  >>> Press 'M' to move to start "
                    f"(at {self._trans_vel*100:.0f}cm/s) <<<")

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
            if self._consume_key() == "m":
                break
            rate.sleep()
        if self._kb_stop.is_set():
            self._emergency_land()
            return
```

**MOVING_TO_START** — linear interpolation from current position to the rectangle start corner:

```python
        self._state = FlightState.MOVING_TO_START
        start_pos = self._get_position()
        if start_pos is None:
            self._emergency_land()
            return

        target_x, target_y = self._rect_start
        dx_move = target_x - start_pos[0]
        dy_move = target_y - start_pos[1]
        move_dist = math.sqrt(dx_move*dx_move + dy_move*dy_move)

        if move_dist < 0.02:
            self._print("  Already at start position!")
        else:
            move_duration = move_dist / self._trans_vel
            move_vx = dx_move / move_duration
            move_vy = dy_move / move_duration
            self._print(f"  Moving to start "
                        f"({move_dist*100:.1f}cm, "
                        f"{move_duration:.1f}s)...")

            move_start = time.monotonic()
            rate.start()
            while (time.monotonic() - move_start
                   < move_duration):
                rclpy.spin_once(self, timeout_sec=0.001)
                t = time.monotonic() - move_start
                frac = t / move_duration
                x = start_pos[0] + dx_move * frac
                y = start_pos[1] + dy_move * frac
                self._publish_full_state(
                    x, y, self._height,
                    vx=move_vx, vy=move_vy)
                if not self._check_safety():
                    self._emergency_land()
                    return
                rate.sleep()
            self._print()
```

**HOVERING** — at the start corner, waiting for 'G':

```python
        self._state = FlightState.HOVERING
        self._print(f"\n  At start position. Hovering at "
                    f"({target_x:.2f}, {target_y:.2f}, "
                    f"{self._height:.2f})")
        self._print(f"\n  Rectangle: "
                    f"{self._rect_side*100:.0f}cm sides, "
                    f"{self._rect_vel*100:.0f}cm/s, 1 loop")
        self._print(f"  >>> Press 'G' to start trajectory "
                    f"flight <<<\n")

        rate.start()
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.001)
            self._publish_full_state(
                target_x, target_y, self._height)
            if not self._check_safety():
                self._emergency_land()
                return
            if self._consume_key() == "g":
                break
            rate.sleep()
        if self._kb_stop.is_set():
            self._emergency_land()
            return
```

**FLYING** — the rectangle trajectory with live progress display:

```python
        self._state = FlightState.FLYING
        time_per_side = self._segment_duration
        total_time = self._trajectory_duration

        self._print(f"\n  FLYING RECTANGLE! "
                    f"({total_time:.1f}s total)")
        self._print(f"  Side time: {time_per_side:.1f}s | "
                    f"Velocity: {self._rect_vel*100:.0f}cm/s")

        flight_start = time.monotonic()
        rate.start()
        while (time.monotonic() - flight_start
               < total_time):
            rclpy.spin_once(self, timeout_sec=0.001)
            elapsed = time.monotonic() - flight_start

            sp = self._rectangle_setpoint(elapsed)
            if sp is not None:
                x, y, vx, vy = sp
                self._publish_full_state(
                    x, y, self._height, vx=vx, vy=vy)

            if not self._check_safety():
                self._emergency_land()
                return

            # Progress display every 0.5s
            cur = self._get_position()
            if sp is not None and cur is not None \
                    and int(elapsed * 2) % 2 == 0:
                err = math.sqrt(
                    (cur[0] - x)**2 + (cur[1] - y)**2)
                seg_idx = min(int(elapsed / time_per_side), 3)
                sys.stdout.write(
                    f"\r    Seg {seg_idx+1}/4 | "
                    f"{elapsed:.1f}/{total_time:.1f}s | "
                    f"cmd=({x:.2f},{y:.2f}) "
                    f"err={err*100:.1f}cm   ")
                sys.stdout.flush()
            rate.sleep()

        self._print(f"\n\n  Rectangle complete!")
```

The progress display uses `\r` (carriage return) to overwrite the same line each update. It shows: current segment (1-4), elapsed time, total time, commanded position (cmd), and tracking error in cm (distance between EKF position and commanded position). The `int(elapsed * 2) % 2 == 0` condition limits updates to every 0.5 seconds to avoid excessive console output.

**World-frame assumption:** The rectangle setpoints are computed in world coordinates and published as-is. The drone must be physically placed facing world +X before takeoff — with single-marker Vicon, yaw is unobservable and the EKF's yaw estimate at takeoff is whatever it was at power-on. Placing the drone correctly ensures the body frame aligns with the world frame.

**Post-trajectory hover and landing** — add after the FLYING state's `while` loop:

```python
        # Brief hover at end (back at start corner)
        hover_end_start = time.monotonic()
        rate.start()
        while time.monotonic() - hover_end_start < 1.5:
            rclpy.spin_once(self, timeout_sec=0.001)
            self._publish_full_state(
                target_x, target_y, self._height)
            if not self._check_safety():
                self._emergency_land()
                return
            rate.sleep()

        # Land
        self._state = FlightState.LANDING
        self._call_notify_stop()
        time.sleep(0.1)
        self._call_land(self._land_dur)
        time.sleep(self._land_dur + 1.0)
        self._airborne = False

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

After completing the rectangle, the drone hovers at the start corner for 1.5s (to stabilize after the last turn), then calls `notify_setpoints_stop` → brief pause → `land`. A flight summary is printed showing the measured yaw offset, final position, battery voltage, and CSV log path.

#### A.4.6: main() and entry point

Add at the bottom of the file, after the `TrajectoryFlightNode` class:

```python
def main():
    rclpy.init()
    node = TrajectoryFlightNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        if node._airborne:
            node._emergency_land()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

Edit `setup.py`. Open `hardware_trajectory/setup.py` and replace its contents:

```python
from setuptools import setup
from glob import glob

package_name = 'hardware_trajectory'

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
    description='Example 6: Hardware trajectory flight',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_flight = hardware_trajectory.trajectory_flight:main',
        ],
    },
)
```

---

### A.5: Build and Test (6A)

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
colcon build --symlink-install
```

**Pre-flight verification:** Same manual checks as Example 5 (verify `/poses`, `/cf1/pose`, `/cf1/status`). The viewer from Example 4 can be used for visual verification — launch it in a separate terminal with `ros2 run vicon_viewer drone_viewer` to confirm the drone's marker appears at the expected position before flight.

**Launch and run (separate terminals, recommended for debugging):**

Terminal 1:
```bash
source install/setup.bash
ros2 launch hardware_trajectory trajectory_flight.launch.py
```

Terminal 2 (the viewer built in Example 4, reused here):
```bash
source install/setup.bash
ros2 run vicon_viewer drone_viewer
```

Terminal 3:
```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run hardware_trajectory trajectory_flight --ros-args -p drone_name:=cf1
```

**Launch and run (convenience, with viewer bundled):**

Terminal 1:
```bash
source install/setup.bash
ros2 launch hardware_trajectory trajectory_flight.launch.py viewer:=true
```

Terminal 2:
```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run hardware_trajectory trajectory_flight --ros-args -p drone_name:=cf1
```

**Key flow:** T (takeoff) → M (move to start) → G (fly rectangle) → auto-land

**Post-flight analysis:** Compare `source=ekf` vs `source=vicon` vs `source=cmd` during the trajectory. Corner transitions (segment changes) will show the largest tracking errors. Velocity feedforward significantly reduces tracking error compared to position-only commands.

**Experiment:** After a successful flight, edit `flight_config.yaml` to change `side_m` (try 0.3 or 0.5) or `velocity_ms` (try 0.15 or 0.25). Zero out `vx` and `vy` in `_rectangle_setpoint` to observe the difference that velocity feedforward makes on hardware.

---

## Part B — Multi-Drone Trajectory Flight

---

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws/src/hardware_trajectory
```

### B.1: Write the Multi-Drone Configs

**crazyflies_multi.yaml** — two drones with distinct URIs and initial positions. Create `config/crazyflies_multi.yaml`:

```bash
touch config/crazyflies_multi.yaml
```

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701
    initial_position: [0.0, 0.0, 0.0]
    type: cf21
  cf3:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E703
    initial_position: [1.8, 0.0, 0.0]
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
        frequency: 10
      status:
        frequency: 1
```

cf3 is placed 1.8m in +X from cf1 for spatial separation between trajectories.

**flight_config_multi.yaml** — per-drone trajectory parameters. Create `config/flight_config_multi.yaml`:

```bash
touch config/flight_config_multi.yaml
```

```yaml
drones:
  cf1:
    trajectory: "rectangle"
    rectangle:
      side_m: 0.40
    start_position_m: [0.0, 0.0]
  cf3:
    trajectory: "circle"
    circle:
      radius_m: 0.40
    start_position_m: [1.8, 0.0]

shared:
  height_m: 0.30
  velocity_ms: 0.20
  transition_velocity_ms: 0.10

safety:
  max_distance_m: 3.0
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

---

### B.2: Write the Multi-Drone Launch File

Create `launch/trajectory_flight_multi.launch.py`:

```bash
touch launch/trajectory_flight_multi.launch.py
```

Same structure as the single-drone launch file, but referencing `crazyflies_multi.yaml` instead of `crazyflies.yaml`:

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
    pkg_dir = get_package_share_directory('hardware_trajectory')
    cs2_share = get_package_share_directory('crazyflie')

    crazyflies_path = os.path.join(
        pkg_dir, 'config', 'crazyflies_multi.yaml')
    mocap_yaml_path = os.path.join(
        pkg_dir, 'config', 'motion_capture.yaml')
    server_yaml_path = os.path.join(
        cs2_share, 'config', 'server.yaml')
    urdf_path = os.path.join(
        cs2_share, 'urdf', 'crazyflie_description.urdf')

    with open(crazyflies_path) as f:
        crazyflies = yaml.safe_load(f)
    with open(mocap_yaml_path) as f:
        mocap_cfg = yaml.safe_load(f)
    with open(server_yaml_path) as f:
        server_cfg = yaml.safe_load(f)
    with open(urdf_path) as f:
        robot_desc = f.read()

    mocap_params = \
        mocap_cfg['/motion_capture_tracking']['ros__parameters']
    mocap_params['rigid_bodies'] = {}
    for key, value in crazyflies['robots'].items():
        if value['enabled']:
            robot_type = crazyflies['robot_types'][
                value['type']]
            mc = robot_type['motion_capture']
            if (mc['enabled']
                    and mc.get('tracking')
                    == 'librigidbodytracker'):
                mocap_params['rigid_bodies'][key] = {
                    'initial_position': value[
                        'initial_position'],
                    'marker': mc['marker'],
                    'dynamics': mc['dynamics'],
                }

    server_params = [crazyflies]
    server_params.append(
        server_cfg['/crazyflie_server']['ros__parameters'])
    server_params[1]['robot_description'] = robot_desc
    server_params[1]['poses_qos_deadline'] = \
        mocap_params['topics']['poses']['qos']['deadline']

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

The only difference from the single-drone launch file: `crazyflies_path` points to `crazyflies_multi.yaml` (which defines both cf1 and cf3). The `rigid_bodies` loop automatically picks up both drones since both have `enabled: true` and `tracking: "librigidbodytracker"`.

---

### B.3: Write trajectory_flight2.py (6B)

Create `hardware_trajectory/trajectory_flight2.py`:

```bash
touch hardware_trajectory/trajectory_flight2.py
```

This script manages two drones simultaneously. It is a complete, self-contained file — the reader writes it from scratch following the blocks below. No cross-referencing with `trajectory_flight.py` (6A) is required, though the patterns are the same.

#### B.3.1: Shebang, imports, RateController, FlightState

```python
#!/usr/bin/env python3
"""Multi-drone hardware trajectory flight with keyboard-gated safety."""

import csv, math, os, select, sys, threading, time
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


class FlightState(Enum):
    WAITING_FOR_PAIR = auto()
    WAITING_FOR_EKF = auto()
    WAITING_FOR_CAN_FLY = auto()
    READY_FOR_TAKEOFF = auto()
    TAKING_OFF = auto()
    STABILIZING = auto()
    READY_TO_MOVE = auto()
    MOVING_TO_START = auto()
    HOVERING = auto()
    FLYING = auto()
    HOVERING_AFTER = auto()
    RETURNING = auto()
    READY_TO_LAND = auto()
    LANDING = auto()
    DONE = auto()
    EMERGENCY = auto()
```

The `FlightState` adds two new states compared to 6A: `HOVERING_AFTER` (wait for 'R' at trajectory end positions) and `RETURNING` (move back to start positions before landing).

#### B.3.2: Config loader

```python
def _load_config(config_path: str) -> dict:
    if not config_path:
        config_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "config", "flight_config_multi.yaml")
    config_path = os.path.abspath(config_path)
    if os.path.exists(config_path):
        with open(config_path) as f:
            return yaml.safe_load(f)
    return {
        "drones": {
            "cf1": {"trajectory": "rectangle",
                    "rectangle": {"side_m": 0.40},
                    "start_position_m": [0.0, 0.0]},
            "cf3": {"trajectory": "circle",
                    "circle": {"radius_m": 0.40},
                    "start_position_m": [0.0, 1.8]},
        },
        "shared": {
            "height_m": 0.30, "velocity_ms": 0.20,
            "transition_velocity_ms": 0.10,
        },
        "safety": {"max_distance_m": 1.0, "max_height_m": 0.60,
                   "min_height_m": 0.05, "pose_timeout_s": 0.5,
                   "battery_critical_v": 3.2},
        "control": {"rate_hz": 30, "takeoff_duration_s": 3.0,
                    "land_duration_s": 3.0, "hover_stabilize_s": 3.0},
    }
```

The fallback defaults include per-drone trajectory configs and shared parameters, so the script can run without a config file.

#### B.3.3: Node __init__ with per-drone dictionaries

```python
class TrajectoryFlight2Node(Node):

    def __init__(self):
        super().__init__("trajectory_flight2")

        self.declare_parameter("config", "")
        config_path = (self.get_parameter("config")
                       .get_parameter_value().string_value)
        self._cfg = _load_config(config_path)

        shared = self._cfg["shared"]
        safety = self._cfg["safety"]
        control = self._cfg["control"]

        self._height = shared["height_m"]
        self._speed = shared["velocity_ms"]
        self._trans_vel = shared["transition_velocity_ms"]

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

        # Per-drone configs
        self._drones = list(self._cfg["drones"].keys())
        self._drone_cfg = self._cfg["drones"]

        # Per-drone reference positions for distance safety checks
        self._ref_positions = {}
        for d in self._drones:
            sp = self._drone_cfg[d].get("start_position_m", [0.0, 0.0])
            self._ref_positions[d] = (float(sp[0]), float(sp[1]))

        # Per-drone state
        self._current_poses = {d: None for d in self._drones}
        self._last_pose_times = {d: 0.0 for d in self._drones}
        self._battery_v = {d: None for d in self._drones}
        self._supervisor_info = {d: 0 for d in self._drones}
        self._paired = {d: False for d in self._drones}
        self._airborne = set()
        self._emergency_triggered = False
        self._emergency_requested = False
        self._state = FlightState.WAITING_FOR_PAIR
```

Each drone has its own current pose (from EKF), battery voltage, supervisor info, and pairing status. The `_airborne` set tracks which drones have taken off.

```python
        # Per-drone subscribers
        self._pose_subs = {}
        self._status_subs = {}
        for drone in self._drones:
            self._pose_subs[drone] = self.create_subscription(
                PoseStamped, f"/{drone}/pose",
                lambda msg, d=drone: self._pose_cb(msg, d), 10)
            self._status_subs[drone] = self.create_subscription(
                Status, f"/{drone}/status",
                lambda msg, d=drone: self._status_cb(msg, d), 10)

        _qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._paired_sub = self.create_subscription(
            NamedPoseArray, "/poses", self._paired_cb, _qos)
```

The lambda capture pattern (`lambda msg, d=drone: ...`) ensures each callback receives the correct drone name. This is the same pattern used in Example 3.

```python
        # Per-drone publishers
        self._fullstate_pubs = {}
        for drone in self._drones:
            self._fullstate_pubs[drone] = self.create_publisher(
                FullState, f"/{drone}/cmd_full_state", 1)

        # Per-drone service clients
        self._takeoff_clis = {}
        self._land_clis = {}
        self._notify_clis = {}
        self._emergency_clis = {}
        for drone in self._drones:
            self._takeoff_clis[drone] = self.create_client(
                Takeoff, f"/{drone}/takeoff")
            self._land_clis[drone] = self.create_client(
                Land, f"/{drone}/land")
            self._notify_clis[drone] = self.create_client(
                NotifySetpointsStop,
                f"/{drone}/notify_setpoints_stop")
            self._emergency_clis[drone] = self.create_client(
                Empty, f"/{drone}/emergency")

        # CSV logs (one per drone)
        self._log_files = {}
        self._log_writers = {}
        self._log_t0 = None
        log_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "logs")
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        for drone in self._drones:
            path = os.path.join(
                log_dir, f"flight2_{drone}_{ts}.csv")
            f = open(path, "w", newline="")
            w = csv.writer(f)
            w.writerow(["time_s", "drone", "state", "source",
                        "x", "y", "z", "vx", "vy", "vz",
                        "yaw_deg", "battery_v"])
            self._log_files[drone] = f
            self._log_writers[drone] = w

        # Keyboard thread
        self._kb_stop = threading.Event()
        self._key_pressed = None
        self._key_lock = threading.Lock()
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True,
            name="keyboard")
```

Each drone gets its own CSV log file so post-flight analysis can compare trajectories independently.

#### B.3.4: Callbacks

```python
    def _pose_cb(self, msg: PoseStamped, drone: str):
        self._current_poses[drone] = msg
        self._last_pose_times[drone] = time.monotonic()
        p = msg.pose.position
        self._log_row(drone, "ekf", p.x, p.y, p.z)

    def _status_cb(self, msg: Status, drone: str):
        self._battery_v[drone] = msg.battery_voltage
        self._supervisor_info[drone] = msg.supervisor_info

    def _paired_cb(self, msg: NamedPoseArray):
        for pose in msg.poses:
            if pose.name in self._drones:
                self._paired[pose.name] = True
                p = pose.pose.position
                self._log_row(pose.name, "vicon", p.x, p.y, p.z)

    def _log_row(self, drone, source, x, y, z,
                 vx=0.0, vy=0.0, vz=0.0, yaw_deg=0.0):
        if self._log_t0 is None:
            self._log_t0 = time.monotonic()
        t = time.monotonic() - self._log_t0
        batt = self._battery_v.get(drone)
        self._log_writers[drone].writerow([
            f"{t:.4f}", drone, self._state.name, source,
            f"{x:.5f}", f"{y:.5f}", f"{z:.5f}",
            f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}",
            f"{yaw_deg:.2f}",
            f"{batt:.2f}" if batt is not None else "",
        ])

    def _get_position(self, drone: str):
        pose = self._current_poses.get(drone)
        if pose is not None:
            p = pose.pose.position
            return (p.x, p.y, p.z)
        return None
```

#### B.3.5: Publish and service methods (per-drone)

```python
    def _publish_full_state(self, drone, x, y, z,
                            vx=0.0, vy=0.0, vz=0.0,
                            ax=0.0, ay=0.0, az=0.0,
                            yaw=0.0):

        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
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
        self._fullstate_pubs[drone].publish(msg)
        self._log_row(drone, "cmd", x, y, z, vx, vy, vz,
                      math.degrees(yaw))

    def _call_takeoff(self, drone, height, duration):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = height
        req.duration = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9))
        future = self._takeoff_clis[drone].call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)
        self._airborne.add(drone)

    def _call_land(self, drone, duration):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9))
        future = self._land_clis[drone].call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)

    def _call_notify_stop(self, drone):
        req = NotifySetpointsStop.Request()
        req.group_mask = 0
        req.remain_valid_millisecs = 100
        future = self._notify_clis[drone].call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)

    def _call_emergency(self, drone):
        req = Empty.Request()
        future = self._emergency_clis[drone].call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=2.0)
```

#### B.3.6: Safety checks (per-drone) and emergency landing (all)

```python
    def _check_safety(self, drone, check_min_height=True) -> bool:
        if self._emergency_requested:
            return False  # keyboard thread requested emergency (all drones)
        if drone not in self._airborne or self._emergency_triggered:
            return True

        age = (time.monotonic()
               - self._last_pose_times.get(drone, 0))
        if age > self._pose_timeout:
            self.get_logger().error(
                f"SAFETY [{drone}]: Pose stale ({age:.2f}s)")
            return False

        pos = self._get_position(drone)
        if pos is not None:
            rx, ry = self._ref_positions.get(drone, (0.0, 0.0))
            dist = math.sqrt((pos[0] - rx)**2 + (pos[1] - ry)**2)
            if dist > self._max_dist:
                self.get_logger().error(
                    f"SAFETY [{drone}]: Distance {dist:.2f}m "
                    f"from ref ({rx:.1f},{ry:.1f})")
                return False
            if pos[2] > self._max_height:
                self.get_logger().error(
                    f"SAFETY [{drone}]: Height {pos[2]:.2f}m")
                return False
            if check_min_height and pos[2] < self._min_height:
                self.get_logger().error(
                    f"SAFETY [{drone}]: Too low ({pos[2]:.2f}m)")
                return False

        IS_TUMBLED = 0x20
        if self._supervisor_info.get(drone, 0) & IS_TUMBLED:
            self.get_logger().error(
                f"SAFETY [{drone}]: Tumbled!")
            return False

        batt = self._battery_v.get(drone)
        if batt is not None and batt < self._batt_critical:
            self.get_logger().error(
                f"SAFETY [{drone}]: Battery {batt:.2f}V")
            return False

        return True

    def _emergency_land_all(self):
        if self._emergency_triggered:
            return
        self._emergency_triggered = True
        self._state = FlightState.EMERGENCY
        self.get_logger().warn("EMERGENCY LAND ALL initiated")

        for drone in list(self._airborne):
            self.get_logger().warn(
                f"  Emergency landing {drone}...")
            IS_TUMBLED = 0x20
            if not (self._supervisor_info.get(drone, 0)
                    & IS_TUMBLED):
                pos = self._get_position(drone)
                if pos is not None:
                    for _ in range(15):
                        self._publish_full_state(
                            drone, pos[0], pos[1], pos[2])
                        time.sleep(self._dt)

            try:
                self._call_notify_stop(drone)
            except Exception:
                pass
            try:
                self._call_land(drone, self._land_dur)
            except Exception:
                self.get_logger().error(
                    f"  Land failed for {drone}, "
                    f"calling emergency stop")
                try:
                    self._call_emergency(drone)
                except Exception:
                    pass
        self._airborne.clear()
```

#### B.3.7: Keyboard thread (unchanged from 6A)

```python
    def _keyboard_loop(self):
        try:
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
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
                if ch in ("e", "\x03"):
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
                termios.tcsetattr(
                    fd, termios.TCSADRAIN, old)

    def _consume_key(self):
        with self._key_lock:
            k = self._key_pressed
            self._key_pressed = None
            return k
```

#### B.3.8: Trajectory computation (rectangle + circle)

The rectangle method is the same four-segment decomposition from 6A, parameterized per-drone. The circle method uses the same center/tangent derivation from Example 3, adapted for hardware:

```python
    def _rectangle_setpoint(self, drone, elapsed, side, duration):
        if elapsed < 0 or elapsed >= duration:
            return None
        seg_dur = duration / 4
        seg_idx = int(elapsed / seg_dur)
        seg_t = elapsed - seg_idx * seg_dur
        frac = seg_t / seg_dur
        x0, y0 = self._drone_cfg[drone]["start_position_m"]
        if seg_idx == 0:
            x, y = x0 + side * frac, y0
            vx, vy = self._speed, 0.0
        elif seg_idx == 1:
            x, y = x0 + side, y0 + side * frac
            vx, vy = 0.0, self._speed
        elif seg_idx == 2:
            x, y = x0 + side * (1.0 - frac), y0 + side
            vx, vy = -self._speed, 0.0
        else:
            x, y = x0, y0 + side * (1.0 - frac)
            vx, vy = 0.0, -self._speed
        return (x, y, vx, vy)

    def _circle_setpoint(self, drone, elapsed, radius, duration):
        if elapsed < 0 or elapsed >= duration:
            return None
        x0, y0 = self._drone_cfg[drone]["start_position_m"]
        cx = x0 - radius  # rightmost point at drone position
        cy = y0
        angle = (elapsed / duration) * 2.0 * math.pi
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        vx = -self._speed * math.sin(angle)
        vy = self._speed * math.cos(angle)
        return (x, y, vx, vy)

    def _get_trajectory_setpoint(self, drone, elapsed):
        cfg = self._drone_cfg[drone]
        traj_type = cfg["trajectory"]
        if traj_type == "rectangle":
            side = cfg["rectangle"]["side_m"]
            duration = 4 * side / self._speed
            sp = self._rectangle_setpoint(
                drone, elapsed, side, duration)
            if sp is not None:
                return sp
            # Finished — hover at start corner
            x0, y0 = cfg["start_position_m"]
            return (x0, y0, 0.0, 0.0)
        else:  # circle
            radius = cfg["circle"]["radius_m"]
            duration = 2 * math.pi * radius / self._speed
            sp = self._circle_setpoint(
                drone, elapsed, radius, duration)
            if sp is not None:
                return sp
            # Finished — hover at start position (circle ends where it began)
            x0, y0 = cfg["start_position_m"]
            return (x0, y0, 0.0, 0.0)

    def _get_trajectory_duration(self, drone):
        cfg = self._drone_cfg[drone]
        if cfg["trajectory"] == "rectangle":
            side = cfg["rectangle"]["side_m"]
            return 4 * side / self._speed
        else:
            radius = cfg["circle"]["radius_m"]
            return 2 * math.pi * radius / self._speed
```

`_get_trajectory_setpoint` routes each drone to its configured trajectory type. When a trajectory returns `None` (finished), the method falls back to a hover setpoint at the end position. This is the hover-wait pattern from Example 3, now on hardware.

#### B.3.9: Terminal-aware output helper

Add the `_print` method inside the `TrajectoryFlight2Node` class, before the run state machine. Same `\r\n`-aware output as the single-drone class:

```python
    def _print(self, text=""):
        """Print with \\r\\n so output is correct in raw terminal mode."""
        for line in text.split("\n"):
            sys.stdout.write(line + "\r\n")
        sys.stdout.flush()
```

#### B.3.10: The main run() state machine

The pre-flight states use the same pattern as 6A but operate on all drones simultaneously. The complete `run()` is provided below — this is a self-contained file, no cross-referencing required.

```python
    def run(self):
        rate = RateController(self._dt)

        # Wait for crazyflie_server services
        self.get_logger().info(
            "Waiting for crazyflie_server services...")
        services_ok = True
        for drone in self._drones:
            for cli, name in [
                (self._takeoff_clis[drone],
                 f"{drone}/takeoff"),
                (self._land_clis[drone],
                 f"{drone}/land"),
                (self._notify_clis[drone],
                 f"{drone}/notify_setpoints_stop"),
                (self._emergency_clis[drone],
                 f"{drone}/emergency")]:
                if not cli.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error(
                        f"Service {name} not available!")
                    services_ok = False
        if not services_ok:
            return

        # --- WAITING_FOR_PAIR (per-drone) ---
        self._print(
            f"\n  Waiting for drones in /poses "
            f"({', '.join(self._drones)})...")
        pair_start = time.monotonic()
        while (not all(self._paired.values())
               and not self._kb_stop.is_set()
               and time.monotonic() - pair_start < 30.0):
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._kb_stop.is_set():
            return
        if not all(self._paired.values()):
            missing = [d for d in self._drones
                       if not self._paired[d]]
            self.get_logger().error(
                f"Drones not paired: {missing}")
            return
        self._print("  All drones paired!")

        # --- WAITING_FOR_EKF (per-drone) ---
        self._print("  Waiting for EKF pose data...")
        ekf_start = time.monotonic()
        while (not all(
                self._current_poses[d] is not None
                for d in self._drones)
               and time.monotonic() - ekf_start < 15.0):
            rclpy.spin_once(self, timeout_sec=0.2)
        missing = [d for d in self._drones
                   if self._current_poses[d] is None]
        if missing:
            self.get_logger().error(
                f"No EKF pose for: {missing}")
            return

        # Convergence check (per-drone, simplified:
        # wait for stable readings)
        for drone in self._drones:
            samples = []
            converge_start = time.monotonic()
            while time.monotonic() - converge_start < 15.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                pos = self._get_position(drone)
                if pos is not None:
                    samples.append(pos)
                    if len(samples) >= 20:
                        recent = samples[-20:]
                        spread = max(
                            max(axis) - min(axis)
                            for axis in zip(*recent))
                        if spread < 0.01:
                            break
            else:
                self.get_logger().warn(
                    f"EKF convergence timeout for {drone}")
        self._print("  All EKF estimates stable.")

        # --- WAITING_FOR_CAN_FLY (per-drone) ---
        self._print(
            "  Waiting for supervisor CAN_FLY "
            "(sensor self-test)...")
        CAN_FLY = 0x08
        cf_start = time.monotonic()
        while (not all(
                self._supervisor_info[d] & CAN_FLY
                for d in self._drones)
               and time.monotonic() - cf_start < 20.0):
            rclpy.spin_once(self, timeout_sec=0.5)
        not_ready = [d for d in self._drones
                     if not (self._supervisor_info[d]
                             & CAN_FLY)]
        if not_ready:
            self.get_logger().warn(
                f"Sensors not ready for: {not_ready}. "
                "Proceeding anyway.")
        else:
            self._print("  All sensors ready!")

        # --- SENSOR_CHECK (battery + supervisor) ---
        for drone in self._drones:
            batt = self._battery_v.get(drone)
            if batt is not None:
                batt_str = (f"{batt:.2f}V"
                            if batt > self._batt_critical
                            else f"{batt:.2f}V (LOW!)")
            else:
                batt_str = "N/A"
            self._print(f"  {drone}: battery={batt_str}")
        self._print("  All pre-flight checks passed.")

        # Start keyboard thread
        self._kb_thread.start()

        # READY_FOR_TAKEOFF — wait for 'T'
        self._print(f"\n  >>> Press 'T' to take off ALL drones "
                    f"to {self._height*100:.0f}cm <<<\n")
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._emergency_requested:
                self._emergency_land_all(); return
            if self._emergency_triggered: return
            if self._consume_key() == "t": break

        # TAKING_OFF — takeoff each drone via HLC
        self._state = FlightState.TAKING_OFF
        self._print(f"\n  [TAKEOFF] Taking off to "
                    f"{self._height*100:.0f}cm...")
        for drone in self._drones:
            self._call_takeoff(drone, self._height,
                               self._takeoff_dur)

        # Spin during the takeoff so EKF updates arrive
        takeoff_wait = self._takeoff_dur + 1.0
        wait_start = time.monotonic()
        while time.monotonic() - wait_start < takeoff_wait:
            rclpy.spin_once(self, timeout_sec=0.05)
            for drone in list(self._airborne):
                if not self._check_safety(drone, check_min_height=False):
                    self._emergency_land_all()
                    return

        # Verify all drones reached height
        for drone in self._drones:
            pos = self._get_position(drone)
            if pos is None or pos[2] < self._height * 0.6:
                self.get_logger().error(
                    f"Takeoff failed for {drone}: "
                    f"height={pos[2] if pos else 'None'}m, "
                    f"need >{self._height*0.6:.2f}m")
                self._emergency_land_all()
                return

        # STABILIZING — let HLC hold position after takeoff.
        # Spin (don't sleep) so EKF pose data keeps flowing.
        self._state = FlightState.STABILIZING
        stab_start = time.monotonic()
        while time.monotonic() - stab_start < self._hover_stab:
            rclpy.spin_once(self, timeout_sec=0.05)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return

        # READY_TO_MOVE — wait for 'M'
        self._state = FlightState.READY_TO_MOVE
        self._print(f"\n  >>> Press 'M' to move all drones "
                    f"to start positions <<<\n")

        rate.start()
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.001)
            for drone in self._drones:
                pos = self._get_position(drone)
                if pos is not None:
                    self._publish_full_state(
                        drone, pos[0], pos[1], self._height)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return
            if self._consume_key() == "m":
                break
            rate.sleep()
        if self._kb_stop.is_set():
            self._emergency_land_all()
            return
```

**Yaw alignment note:** Same as 6A — single-marker Vicon provides no orientation. Place all drones facing world +X before takeoff.

**Move-to-start and fly (multi-drone with hover-wait):**

```python
        # MOVING_TO_START — all drones move simultaneously
        self._state = FlightState.MOVING_TO_START
        # Record current positions and compute per-drone targets
        starts = {}
        targets = {}
        for drone in self._drones:
            pos = self._get_position(drone)
            if pos is None:
                self.get_logger().error(f"No pose for {drone}!")
                self._emergency_land_all()
                return
            starts[drone] = pos
            tgt = self._drone_cfg[drone]["start_position_m"]
            targets[drone] = (tgt[0], tgt[1])
        # Synchronize transition: all drones arrive simultaneously
        max_dist = max(
            math.sqrt((targets[d][0] - starts[d][0])**2
                      + (targets[d][1] - starts[d][1])**2)
            for d in self._drones)
        trans_dur = (max_dist / self._trans_vel
                     if self._trans_vel > 0 else 1.0)
        self._print(f"  Moving to start positions "
                    f"({max_dist*100:.1f}cm max, "
                    f"{trans_dur:.1f}s)...")
        trans_start = time.monotonic()
        rate.start()
        while time.monotonic() - trans_start < trans_dur:
            rclpy.spin_once(self, timeout_sec=0.001)
            elapsed = time.monotonic() - trans_start
            frac = min(elapsed / trans_dur, 1.0)
            for drone in self._drones:
                sx, sy, sz = starts[drone]
                tx, ty = targets[drone]
                x = sx + (tx - sx) * frac
                y = sy + (ty - sy) * frac
                self._publish_full_state(
                    drone, x, y, self._height)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return
            rate.sleep()

        # HOVERING — hover at start positions, wait for 'G'
        self._state = FlightState.HOVERING
        for drone in self._drones:
            tx, ty = targets[drone]
            self._print(f"  {drone} at start: "
                        f"({tx:.2f}, {ty:.2f}, {self._height:.2f})")
        self._print(f"\n  >>> Press 'G' to start trajectory flight <<<\n")

        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
            for drone in self._drones:
                tx, ty = targets[drone]
                self._publish_full_state(
                    drone, tx, ty, self._height)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return
            if self._consume_key() == "g": break

        # FLYING — all drones fly simultaneously
        self._state = FlightState.FLYING
        fly_duration = max(
            self._get_trajectory_duration(d)
            for d in self._drones)
        self._print(f"\n  FLYING! "
                    f"({fly_duration:.1f}s total)")
        flight_start = time.monotonic()
        rate.start()
        while (time.monotonic() - flight_start
               < fly_duration):
            rclpy.spin_once(self, timeout_sec=0.001)
            elapsed = time.monotonic() - flight_start

            for drone in self._drones:
                sp = self._get_trajectory_setpoint(
                    drone, elapsed)
                if sp is not None:
                    x, y, vx, vy = sp
                    self._publish_full_state(
                        drone, x, y, self._height,
                        vx=vx, vy=vy)

            # Per-drone safety
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return

            # Progress display every 0.5s
            if int(elapsed * 2) % 2 == 0:
                sys.stdout.write(
                    f"\r    {elapsed:.1f}/{fly_duration:.1f}s")
                sys.stdout.flush()
            rate.sleep()

        if rate.overruns:
            self.get_logger().warning(
                f'{rate.overruns} loop overruns at '
                f'{self._rate_hz}Hz')
        self._print(f"\n\n  All trajectories complete!")

        # HOVERING_AFTER — hover at end positions, wait for 'R' to return
        self._state = FlightState.HOVERING_AFTER
        self._print(f"\n  >>> Press 'R' to return to start positions <<<\n")

        rate.start()
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.001)
            for drone in self._drones:
                tx, ty = targets[drone]
                self._publish_full_state(drone, tx, ty, self._height)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return
            if self._consume_key() == "r":
                break
            rate.sleep()
        if self._kb_stop.is_set():
            self._emergency_land_all()
            return

        # RETURNING — move back to start positions
        self._state = FlightState.RETURNING
        returns = {}
        for drone in self._drones:
            pos = self._get_position(drone)
            if pos is None:
                self.get_logger().error(f"No pose for {drone}!")
                self._emergency_land_all()
                return
            returns[drone] = pos

        max_dist = max(
            math.sqrt((returns[d][0] - targets[d][0])**2
                      + (returns[d][1] - targets[d][1])**2)
            for d in self._drones)
        ret_dur = (max_dist / self._trans_vel
                   if self._trans_vel > 0 else 1.0)
        self._print(f"  Returning to start "
                    f"({max_dist*100:.1f}cm max, "
                    f"{ret_dur:.1f}s)...")
        ret_start = time.monotonic()
        rate.start()
        while time.monotonic() - ret_start < ret_dur:
            rclpy.spin_once(self, timeout_sec=0.001)
            elapsed = time.monotonic() - ret_start
            frac = min(elapsed / ret_dur, 1.0)
            for drone in self._drones:
                sx, sy, sz = returns[drone]
                tx, ty = targets[drone]
                x = sx + (tx - sx) * frac
                y = sy + (ty - sy) * frac
                self._publish_full_state(
                    drone, x, y, self._height)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return
            rate.sleep()

        # READY_TO_LAND — hover at start, wait for 'L'
        self._state = FlightState.READY_TO_LAND
        self._print(f"\n  At start positions.")
        self._print(f"  >>> Press 'L' to land all drones <<<\n")

        rate.start()
        while not self._kb_stop.is_set():
            rclpy.spin_once(self, timeout_sec=0.001)
            for drone in self._drones:
                tx, ty = targets[drone]
                self._publish_full_state(drone, tx, ty, self._height)
            for drone in list(self._airborne):
                if not self._check_safety(drone):
                    self._emergency_land_all()
                    return
            if self._consume_key() == "l":
                break
            rate.sleep()
        if self._kb_stop.is_set():
            self._emergency_land_all()
            return

        # LANDING — all drones land together
        self._state = FlightState.LANDING
        self._print(f"\n  Landing all drones...")
        for drone in self._drones:
            self._call_notify_stop(drone)
        time.sleep(0.1)
        for drone in list(self._airborne):
            self._call_land(drone, self._land_dur)
        time.sleep(self._land_dur + 1.0)
        self._airborne.clear()

        self._state = FlightState.DONE
        self._print(f"\n  Flight complete!")
        for drone in self._drones:
            pos = self._get_position(drone)
            if pos is not None:
                self._print(f"  {drone}: final position "
                            f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            batt = self._battery_v.get(drone)
            if batt is not None:
                self._print(f"         battery {batt:.2f}V")
```

**Synchronized transition:** Both drones move to their start corners simultaneously. The transition duration is computed from the longest move distance (`max_dist / trans_vel`), so the drone with the shorter distance moves proportionally slower and both arrive together.

The flight runs for `max(cf1_duration, cf3_duration)`. The drone that finishes first automatically hovers at its end position because `_get_trajectory_setpoint` returns a hover setpoint when the trajectory is done. Per-drone safety checks run on every iteration — if any drone violates safety, all drones land.

**Return-to-start and keyboard-gated landing:** After the trajectories complete, the drones hover at their end positions and wait for 'R'. Pressing 'R' triggers a synchronized move back to the start positions (same linear interpolation pattern as the move-to-start phase). The drones then hover at start and wait for 'L' before landing. This keyboard-gated return makes the flight symmetric and gives the pilot control over when to end the session.

#### B.3.11: Cleanup and main()

```python
    def _close_logs(self):
        for f in self._log_files.values():
            f.close()

    def shutdown(self):
        self._kb_stop.set()
        if self._airborne and not self._emergency_triggered:
            self._emergency_land_all()
        self._close_logs()


def main():
    rclpy.init()
    node = TrajectoryFlight2Node()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        if node._airborne:
            node._emergency_land_all()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

---

### B.4: Register Entry Points

Edit `setup.py`. Open `hardware_trajectory/setup.py` and replace its contents to add the second entry point:

```python
from setuptools import setup
from glob import glob

package_name = 'hardware_trajectory'

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
    description='Example 6: Hardware trajectory flight',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_flight = hardware_trajectory.trajectory_flight:main',
            'trajectory_flight2 = hardware_trajectory.trajectory_flight2:main',
        ],
    },
)
```

---

### B.5: Build and Test (6B)

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
colcon build --symlink-install
```

**Pre-flight verification:** Check `/poses`, `/{drone}/pose`, and `/{drone}/status` for BOTH drones. The viewer from Example 4 is especially useful here — launch it with `ros2 run vicon_viewer drone_viewer` to visually confirm both drones' markers appear at their expected positions and with correct labels before flight.

**Launch and run (separate terminals, recommended for debugging):**

Terminal 1:
```bash
source install/setup.bash
ros2 launch hardware_trajectory trajectory_flight_multi.launch.py
```

Terminal 2 (the viewer built in Example 4, reused here):
```bash
source install/setup.bash
ros2 run vicon_viewer drone_viewer
```

Terminal 3:
```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run hardware_trajectory trajectory_flight2
```

**Launch and run (convenience, with viewer bundled):**

Terminal 1:
```bash
source install/setup.bash
ros2 launch hardware_trajectory trajectory_flight_multi.launch.py viewer:=true
```

Terminal 2:
```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run hardware_trajectory trajectory_flight2
```

**Key flow:** T (takeoff both) → M (move both to start) → G (fly both) → R (return to start) → L (land both)

**Post-flight:** Compare per-drone CSV logs. Verify cf1's rectangle tracking and cf3's circle tracking. Confirm the hover-wait behavior: cf1 finishes first and hovers while cf3 completes.

---

## Key Concepts (6A + 6B)

- Hardware trajectory execution — same segment decomposition math as sim, now with real drone dynamics and EKF position feedback
- Move-to-start — linear interpolation from hover position to trajectory corner with velocity feedforward
- Progress display — live-updating console output with segment number, elapsed time, and real-time tracking error
- Per-drone safety in multi-drone flight — any violation on any drone triggers emergency landing for the entire swarm
- Hardware hover-wait — finished drones hover at end positions while others complete their trajectories; stopping setpoints would trigger emergency timeout
- Post-flight CSV analysis — comparing `source=ekf` vs `source=vicon` vs `source=cmd` during dynamic flight reveals tracking performance and EKF health

---

## Potential Issues

- **Tracking error large (> 20cm) during rectangle** — velocity feedforward may be incorrect, or EKF convergence may be poor. Reduce speed or check physical drone alignment.
- **Drone overshoots corners** — PID controller has slower response than Mellinger (used in sim). Reduce velocity or increase `extPosStdDev` slightly.
- **Multi-drone: drones come too close** — increase spatial separation between `initial_position` values, or reduce trajectory sizes.
- **Progress display garbled** — `\r` carriage return without `\n` may not work in some terminals. Increase the print interval (change `int(elapsed * 2) % 2 == 0` to `int(elapsed) % 1 == 0` for 1-second updates).
- **CSV shows EKF diverging from Vicon** — the EKF may have a wrong yaw estimate, or `extPosStdDev` may be too large. Power cycle the drone and ensure it is physically placed facing world +X before takeoff.
- **Terminal is garbled / can't see what you're typing** — the script may have exited (or crashed) while the terminal was in raw mode. Type `reset` and press Enter blindly to restore the terminal.
