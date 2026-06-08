# Example 4 — Vicon Streaming and Visualization

This example builds a real-time 3D visualization tool for motion capture data. Unlike Examples 1-3 which controlled simulated drones, this example creates a standalone viewer that subscribes to the `/poses` topic and renders drone positions with trails, shadows, labels, and a HUD overlay. A separate fake mocap publisher is also built, enabling testing without Vicon hardware.

The viewer is fully self-contained — all vispy scene building, data bridging, obstacle rendering, and CSV recording are embedded in a single script with no imports from other sub-projects.

This example assumes completion of Examples 1-3. Familiarity with ROS2 package structure, launch files, and topics is expected. New concepts covered: vispy 3D rendering, thread safety between ROS2 callbacks and a GUI event loop, CSV recording, obstacle CSV parsing, and BEST_EFFORT QoS for high-rate sensor data.

> **Please do use AI while going through this tutorial!** Some parts of this tutorial, like the viewer in example 4, include helper functions and overcomplicated settings just for style and completeness and are unrelated to core usage of ROS2 and Crazyflie. Going through the visualization and state machine function pieces can be **Tedious** and unnecessary. Therefore usage of AI to just get the hang of it and focusing on the important parts is strongly advised!

---

## Files Created

```
tutorial_ws/src/vicon_viewer/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   ├── crazyflies.yaml
│   └── motion_capture.yaml
├── launch/
│   └── viewer.launch.py
└── vicon_viewer/
    ├── __init__.py
    ├── drone_viewer.py
    └── fake_mocap.py
```

---

## Section 1: Create the Package

Create the package skeleton inside the shared workspace:

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws/src
ros2 pkg create vicon_viewer --build-type ament_python \
    --dependencies rclpy motion_capture_tracking_interfaces geometry_msgs
cd vicon_viewer
mkdir -p config launch
```

`geometry_msgs` is declared because the code accesses `geometry_msgs/Pose` fields (`.pose.position`, `.pose.orientation`) through `NamedPose` messages — it resolves transitively today, but declaring it keeps the package self-contained (same standard applied in Examples 1-3).

The dependency `motion_capture_tracking_interfaces` provides `NamedPoseArray` and `NamedPose` — the message types used by the `/poses` topic. This package was installed as part of `ros-humble-motion-capture-tracking` in the prerequisites.

---

## Section 2: Write the Configuration Files

Two configuration files are needed: `crazyflies.yaml` for drone definitions (used by both `fake_mocap.py` and the CS2 mocap pipeline) and `motion_capture.yaml` for the Vicon connection (used in Phase 2 hardware testing).

### 2.1: crazyflies.yaml

Create `config/crazyflies.yaml`:

```bash
touch config/crazyflies.yaml
```

This file uses the full CS2 format so it works with both `fake_mocap.py` (which only reads `robots` and `initial_position`) and the real mocap pipeline (which requires `fileversion`, `robot_types`, and `motion_capture` sections). Three drones are configured so the viewer displays multiple markers with distinct colors.

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701
    initial_position: [0.0, 0.0, 0.0]
    type: cf21
  cf2:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E702
    initial_position: [1.0, 0.0, 0.0]
    type: cf21
  cf3:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E703
    initial_position: [0.0, 1.0, 0.0]
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
    enabled: false
```

The `enabled` field controls which drones `fake_mocap.py` publishes in Phase 1. The `initial_position` is the center point around which each drone oscillates. The `uri` values are placeholder radio addresses — they are not used by `fake_mocap`, but the CS2 launch file requires them. For Phase 2 (real Vicon), only the `motion_capture` and `initial_position` fields are used by the mocap pipeline; the drone connection itself is irrelevant.

### 2.2: motion_capture.yaml

Create `config/motion_capture.yaml`:

```bash
touch config/motion_capture.yaml
```

This configures the connection to Vicon Tracker and the single-marker tracking setup. It is used in Phase 2 when testing with real hardware.

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

`type: "vicon"` selects the Vicon DataStream SDK backend. `hostname` must match the IP address of the Vicon Tracker PC (configured in the mocap prerequisites, Part 0 Section 2). `marker_configurations.default_single_marker` defines a single point at (0, 0, 0) — the drone's center. `dynamics_configurations.default` sets physical limits for the tracker's motion model: per-axis velocity and angular velocity limits, absolute roll/pitch angle limits, and a marker-fit fitness threshold. The tracker uses these to predict drone positions between frames.

---

## Section 3: Write fake_mocap.py

Create `vicon_viewer/fake_mocap.py`:

```bash
touch vicon_viewer/fake_mocap.py
```

This is a standalone ROS2 node that publishes simulated marker data to `/poses`, enabling viewer testing without any hardware.

### 3.1: Shebang, imports, and YAML loader

```python
#!/usr/bin/env python3
"""Mock motion capture publisher for testing without Vicon hardware.

Publishes NamedPoseArray to /poses at 100Hz. Reads drone names and
initial positions from crazyflies.yaml.
"""

import argparse
import math
import os
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose
```

**Imports explained:**
- `argparse` — parses command-line arguments (`--config`, `--height`, `--rate`)
- `math` — sine and cosine for sinusoidal motion
- `yaml` — reads the `crazyflies.yaml` config file
- `QoSProfile` and related constants — configure the publisher's Quality of Service to match real mocap data (BEST_EFFORT, VOLATILE, KEEP_LAST, depth=1)
- `NamedPoseArray` — a list of named poses (the message type published on `/poses`)
- `NamedPose` — a single entry in the array: a name string and a `geometry_msgs/Pose`

```python
def load_drones_from_yaml(config_path: str) -> dict:
    """Parse crazyflies.yaml and return {name: [x, y, z]} for enabled drones."""
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    drones = {}
    robots = cfg.get("robots", {})
    for name, robot in robots.items():
        if robot.get("enabled", True):
            pos = robot.get("initial_position", [0.0, 0.0, 0.0])
            drones[name] = pos
    return drones
```

`load_drones_from_yaml` parses the same YAML format used by the hardware examples. It extracts a mapping from drone name to `[x, y, z]` initial position for every enabled drone. Drones with `enabled: false` are skipped.

### 3.2: FakeMocap node class

Add the following class after the `load_drones_from_yaml` function:

```python
class FakeMocap(Node):
    """ROS2 node that publishes simulated marker data to /poses."""

    def __init__(self, drones: dict, height: float, rate_hz: float = 100.0):
        super().__init__("fake_mocap")

        self._drones = drones
        self._height = height
        self._rate_hz = rate_hz

        # Match the QoS that real motion_capture_tracking_node uses
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(NamedPoseArray, "/poses", qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self._start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f"Publishing /poses at {rate_hz}Hz for {len(drones)} drone(s): "
            f"{list(drones.keys())}"
        )
```

**Arguments explained:**
- `drones` — a dict mapping drone name to `[x, y, z]` initial position, from `load_drones_from_yaml`
- `height` — fixed Z coordinate for all drones (default 0.3 = hover above the floor)
- `rate_hz` — publish rate in Hz (default 100, matching real Vicon)

**QoS explained:** `BEST_EFFORT` means messages may be dropped if the network is congested — acceptable for real-time sensor data where a fresh reading is preferred over a delayed one. `VOLATILE` means no persistence (subscribers that join late do not receive old messages). `KEEP_LAST` with `depth=1` keeps only the latest message in the publisher queue.

`create_timer(1.0 / rate_hz, self._publish)` creates a ROS2 timer that calls `_publish` at the specified rate. This is simpler than a manual `while` loop with `RateController` for a publisher-only node.

### 3.3: The _publish method

Add this method inside the `FakeMocap` class:

```python
    def _publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self._start_time

        msg = NamedPoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        for i, (name, initial_pos) in enumerate(self._drones.items()):
            np = NamedPose()
            np.name = name

            # Each drone gets a unique phase offset so they move
            # out of sync, making trails visually distinct
            phase = i * 2.0 * math.pi / max(len(self._drones), 1)

            # Different X and Y frequencies create Lissajous-like patterns
            np.pose.position.x = float(initial_pos[0]) \
                + 0.1 * math.sin(t * 1.0 + phase)
            np.pose.position.y = float(initial_pos[1]) \
                + 0.1 * math.cos(t * 1.4 + phase)
            np.pose.position.z = float(self._height)

            # NaN quaternion = position-only (matching real single-marker Vicon)
            np.pose.orientation.w = float("nan")
            np.pose.orientation.x = float("nan")
            np.pose.orientation.y = float("nan")
            np.pose.orientation.z = float("nan")

            msg.poses.append(np)

        self._pub.publish(msg)
```

**How the motion works:**
- `phase = i * 2π / N` — distributes drones evenly around the phase circle so they move out of sync
- X position oscillates at 1.0 rad/s, Y at 1.4 rad/s — different frequencies create Lissajous curves rather than simple circles, making the motion more visually interesting
- Amplitude of 0.1m keeps drones within a small area for testing
- `float("nan")` for all quaternion fields — real single-marker Vicon provides position only (no orientation). The NaN signals `crazyflie_server` to use position-only EKF updates. The viewer ignores orientation

### 3.4: The main function

Add the `main` function at the bottom of the file:

```python
def main():
    # ROS2 passes extra arguments (--ros-args ...); filter them out
    # so argparse doesn't choke on them
    argv = sys.argv[1:]
    ros_args = []
    clean_args = []
    for i, arg in enumerate(argv):
        if arg == "--ros-args":
            ros_args = argv[i:]
            break
        clean_args.append(arg)

    parser = argparse.ArgumentParser(description="Mock /poses publisher")
    parser.add_argument("--config", type=str, default="",
                        help="Path to crazyflies.yaml")
    parser.add_argument("--height", type=float, default=0.3,
                        help="Fixed Z height for all drones (default: 0.3)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Publish rate in Hz (default: 100)")
    args = parser.parse_args(clean_args)

    # Default config path: <package_dir>/config/crazyflies.yaml
    config_path = args.config
    if not config_path:
        config_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "config", "crazyflies.yaml",
        )
    config_path = os.path.abspath(config_path)

    if not os.path.exists(config_path):
        print(f"Config not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    drones = load_drones_from_yaml(config_path)
    if not drones:
        print("No enabled drones found in config!", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=ros_args if ros_args else None)
    node = FakeMocap(drones, args.height, args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

**ROS2 argument filtering:** `ros2 run` passes `--ros-args ...` to the script. The argparse library does not recognize these arguments. The loop separates ROS arguments from script arguments before parsing so argparse only sees `--config`, `--height`, and `--rate`.

**Default config path:** `os.path.dirname(os.path.realpath(__file__))` resolves to the directory containing the script. Going up one level (`..`) and into `config/` finds `crazyflies.yaml`. `os.path.abspath` normalizes the path.

---

## Section 4: Write drone_viewer.py

Create `vicon_viewer/drone_viewer.py`:

```bash
touch vicon_viewer/drone_viewer.py
```

This is the main file — a full vispy 3D viewer with HUD panels, trails, shadows, drop-lines, labels, a dynamic legend, mouse/keyboard recording controls, and optional obstacle rendering. The visual style mirrors the original Vicon marker visualizer (dark studio floor, two-tone grid, halo/core markers, bright saturated palette) while keeping all `/poses` data in meters.

The file is presented in sequential blocks below. Each block builds on the previous one.

---

### 4.1: Shebang, imports, and theme constants

```python
#!/usr/bin/env python3
"""
3D drone viewer using vispy, subscribing to Crazyswarm2's /poses topic.

Self-contained: all vispy scene building, data bridging, obstacle
rendering, and trajectory logging are embedded in this file.
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray

from vispy import app, scene
from vispy.scene import visuals as scene_visuals
```

**Imports explained:**
- `threading` — `threading.Lock` for thread-safe data sharing between the ROS2 callback thread and the vispy rendering main thread
- `collections.deque` — double-ended queue for trail history. `deque.popleft()` is O(1), making trail pruning efficient
- `numpy` — for vertex and color arrays passed to vispy
- `SingleThreadedExecutor` — ROS2 executor that processes callbacks in the calling thread, used with vispy's timer for cooperative multitasking
- `vispy.app`, `vispy.scene`, `vispy.scene.visuals` — the vispy OpenGL visualization library. `scene_visuals` is used as an alias to make it explicit that `Mesh`, `Line`, `Markers`, `Text`, and `Rectangle` come from vispy

**Theme constants — add after the imports:**

```python
# ---------------------------------------------------------------------------
# Theme — world geometry is in meters, screen values are in pixels
# ---------------------------------------------------------------------------

BG_COLOR        = (0.07, 0.085, 0.115, 1.0)
FLOOR_COLOR     = (0.105, 0.125, 0.155, 1.0)
GRID_MINOR      = (0.18, 0.21, 0.27, 0.55)
GRID_MAJOR      = (0.34, 0.42, 0.55, 0.85)
FLOOR_BORDER    = (0.45, 0.55, 0.72, 0.7)
AXIS_X_RGB      = (1.00, 0.32, 0.36)
AXIS_Y_RGB      = (0.42, 0.95, 0.50)
AXIS_Z_RGB      = (0.40, 0.65, 1.00)
TEXT_PRIMARY    = (0.88, 0.91, 0.96, 1.0)
TEXT_DIM        = (0.55, 0.60, 0.68, 1.0)
TEXT_ACCENT     = (0.55, 0.78, 1.00, 1.0)
PANEL_FILL      = (0.04, 0.06, 0.10, 0.78)
PANEL_BORDER    = (0.22, 0.28, 0.36, 0.9)
ORIGIN_DOT      = (0.78, 0.83, 0.92, 0.9)
EDGE_HIGHLIGHT  = (1.0, 1.0, 1.0, 0.85)

# Scene geometry constants (meters)
DEFAULT_FLOOR_HALF = 3.5       # 7m x 7m floor
DEFAULT_GRID_MINOR = 0.25      # minor grid spacing
DEFAULT_GRID_MAJOR = 1.0       # major grid spacing
DEFAULT_AXIS_LENGTH = 0.5      # length of XYZ axes
DEFAULT_MARKER_SIZE = 11.0     # pixels (screen-space, not meters)
FLOOR_Z_MINOR = 0.0005         # tiny lifts avoid z-fighting with the floor
FLOOR_Z_MAJOR = 0.0006
FLOOR_Z_BORDER = 0.0007
FLOOR_Z_VIS = 0.001            # shadow and drop-line floor projection height
LABEL_Z_OFFSET = 0.07          # height of name labels above each drone
AXIS_LABEL_PAD = 0.13          # distance from axis tip to label text

OBSTACLE_COLOR = (0.6, 0.3, 0.3, 0.15)
OBSTACLE_EDGE_COLOR = (0.8, 0.4, 0.4, 0.4)
OBSTACLE_HEIGHT = 0.4  # meters

# Bright, saturated palette designed to pop against the dark background.
_DRONE_COLORS = [
    (0.30, 0.70, 1.00), (1.00, 0.40, 0.45), (0.45, 0.95, 0.50),
    (1.00, 0.78, 0.25), (0.85, 0.50, 1.00), (0.30, 0.95, 0.85),
    (1.00, 0.55, 0.30), (0.95, 0.45, 0.75), (0.55, 0.90, 1.00),
    (0.90, 0.92, 0.40), (0.60, 0.75, 1.00), (1.00, 0.65, 0.55),
    (0.35, 0.85, 0.65), (0.80, 0.65, 1.00), (0.95, 0.85, 0.55),
    (0.40, 0.80, 0.95), (1.00, 0.50, 0.85), (0.70, 1.00, 0.55),
    (0.95, 0.70, 0.40), (0.55, 0.95, 1.00),
]
```

All colors are RGBA tuples with values in the range 0.0–1.0. The `_DRONE_COLORS` palette has 20 distinct saturated colors — enough for up to 20 drones. The scene geometry constants separate world-space values (meters) from screen-space values (pixels): floor size, grid spacing, axes, camera distance, and label offset are in meters; marker size, line width, font size, HUD dimensions, and button dimensions remain in pixels.

---

### 4.2: Utility functions

Add after the theme constants. These provide deterministic per-drone colors and gradient arrays for trail and shadow rendering:

```python
def drone_color(name: str) -> tuple[float, float, float, float]:
    """Return a stable RGBA color for a drone name."""
    # Do not use Python's built-in hash(); it is intentionally salted per run.
    import hashlib
    idx = int(hashlib.md5(name.encode("utf-8")).hexdigest(), 16) % len(_DRONE_COLORS)
    return _DRONE_COLORS[idx] + (1.0,)


def trail_color_array(rgb, n_points: int,
                      max_alpha: float = 0.95,
                      gamma: float = 1.4) -> np.ndarray:
    """Per-vertex RGBA where oldest point is transparent and newest is bright."""
    colors = np.tile(np.array([*rgb, 0.0], dtype=np.float32), (n_points, 1))
    t = np.linspace(0.0, 1.0, n_points, dtype=np.float32)
    colors[:, 3] = max_alpha * (t ** gamma)
    return colors


def shadow_color_array(n_points: int, max_alpha: float = 0.30) -> np.ndarray:
    """Greyscale alpha-fade for floor projections."""
    colors = np.zeros((n_points, 4), dtype=np.float32)
    t = np.linspace(0.0, 1.0, n_points, dtype=np.float32)
    colors[:, 3] = max_alpha * (t ** 1.6)
    return colors
```

`drone_color` computes a deterministic MD5 hash of the drone name to pick a palette index — the same name always gets the same color, even across sessions. (Python's built-in `hash()` is salted per-process and would give different colors on each launch.) `trail_color_array` creates a gradient from transparent (oldest) to bright (newest) using a gamma curve, making trail direction visually obvious. `shadow_color_array` does the same for floor projection shadows at lower opacity with a steeper gamma (1.6 vs 1.4).

---

### 4.3: DataBridge class

Add after the utility functions. This is the thread-safe data store between the ROS2 subscriber callback and the vispy display timer:

```python
class DataBridge:
    """Thread-safe store for drone positions and time-stamped trails."""

    def __init__(self, trail_duration: float = 30.0):
        self.trail_duration = trail_duration
        self.lock = threading.Lock()
        self.current: dict[str, tuple[float, float, float]] = {}
        self.trails: dict[str, deque] = {}
        self.frame_number = 0
        self.frame_rate = 0.0
        self.num_drones = 0

    def update(self, drones: dict[str, tuple[float, float, float]],
               frame_number: int, frame_rate: float):
        """Called from ROS2 callback. Stores latest positions."""
        now = time.time()
        with self.lock:
            self.current = dict(drones)
            self.frame_number = frame_number
            self.frame_rate = frame_rate
            self.num_drones = len(drones)

            for name, pos in drones.items():
                if name not in self.trails:
                    self.trails[name] = deque()
                self.trails[name].append((now, pos[0], pos[1], pos[2]))

            # Remove trail history for drones that disappeared
            for name in list(self.trails):
                if name not in drones:
                    self.trails[name].clear()
                    del self.trails[name]

    def snapshot(self):
        """Called from vispy timer. Returns a consistent snapshot."""
        now = time.time()
        cutoff = now - self.trail_duration
        with self.lock:
            trails_out: dict[str, list[tuple[float, float, float]]] = {}
            for name, trail in self.trails.items():
                # Prune entries older than trail_duration
                while trail and trail[0][0] < cutoff:
                    trail.popleft()
                if trail:
                    trails_out[name] = [(t[1], t[2], t[3]) for t in trail]
            return (
                dict(self.current),
                trails_out,
                self.frame_number,
                self.frame_rate,
                self.num_drones,
            )
```

**Thread safety explained:** `threading.Lock()` protects `current` and `trails` from concurrent modification. `update()` is called from the ROS2 subscriber callback (driven by the `_spin_ros` timer on the main thread). `snapshot()` is called from the `_update_display` timer (also on the main thread). Since both timers run on the same thread via `SingleThreadedExecutor`, the lock is not strictly necessary in this design — but it is kept as a safeguard for potential future multi-threading and to document the intended data ownership boundary.

**Trail pruning:** `while trail and trail[0][0] < cutoff: trail.popleft()` removes points older than `trail_duration` seconds. `deque.popleft()` is O(1). Without pruning, the trail history would grow indefinitely, consuming memory.

---

### 4.4: TrajectoryLogger class

Add after `DataBridge`. Handles CSV recording with keyboard/mouse toggle:

```python
class TrajectoryLogger:
    """CSV logger for /poses trajectories."""

    def __init__(self, output_dir: str = "recordings"):
        self._output_dir = output_dir
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        self._recording = False
        self._frame_count = 0
        self._start_time = 0.0
        self._path = ""

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def path(self) -> str:
        return self._path

    @property
    def start_time(self) -> float:
        return self._start_time

    def start(self) -> str:
        """Begin recording. Returns the CSV file path."""
        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(self._output_dir, f"trajectory_{ts}.csv")
        with self._lock:
            self._file = open(self._path, "w", newline="")
            self._writer = csv.writer(self._file)
            self._writer.writerow([
                "timestamp_s", "frame", "drone_name", "x_m", "y_m", "z_m",
            ])
            self._frame_count = 0
            self._start_time = time.time()
            self._recording = True
        return self._path

    def stop(self) -> str:
        """Stop recording. Returns the CSV file path."""
        with self._lock:
            self._recording = False
            if self._file is not None:
                self._file.close()
                self._file = None
                self._writer = None
            return self._path

    def log_frame(self, drones: dict[str, tuple[float, float, float]],
                  frame_number: int) -> None:
        """Write one frame of position data (if recording)."""
        with self._lock:
            if not self._recording or self._writer is None:
                return
            t = time.time() - self._start_time
            for name, (x, y, z) in drones.items():
                self._writer.writerow([
                    f"{t:.6f}", frame_number, name,
                    f"{x:.5f}", f"{y:.5f}", f"{z:.5f}",
                ])
            self._frame_count += 1
```

The CSV format is: relative timestamp (seconds since recording started), frame number (from the mocap stream), drone name, and X/Y/Z position in meters. `start()` creates a timestamped filename in the `recordings/` directory. `stop()` closes the file. `log_frame()` writes one row per drone per frame — called from the display timer only when recording is active.

---

### 4.5: build_vispy_scene — 3D scene

Add after `TrajectoryLogger`. This function constructs the complete vispy scene and returns a dictionary of visual objects. It is called once at startup; individual visuals are updated in-place each frame.

```python
def build_vispy_scene() -> dict:
    """Build the full vispy scene and return mutable visual objects."""
    canvas = scene.SceneCanvas(
        keys="interactive",
        size=(1280, 800),
        title="Drone Viewer",
        bgcolor=BG_COLOR,
    )

    view = canvas.central_widget.add_view()
    view.camera = scene.cameras.TurntableCamera(
        elevation=22,
        azimuth=42,
        center=(0, 0, 0.4),
        distance=4.5,
        scale_factor=4.5,
        fov=45,
        up="+z",
    )
```

`SceneCanvas` — the main vispy window. `keys="interactive"` enables built-in mouse interaction (drag to rotate, scroll to zoom). `TurntableCamera` orbits around a center point: `elevation=22` (22 degrees above the horizon), `azimuth=42` (42 degrees rotation around Z), `up="+z"` (Z-up convention matching Vicon and CS2), `distance=4.5` (4.5 meters from center), `fov=45` (field of view in degrees).

**Floor, grid, and axes — continue inside `build_vispy_scene`:**

```python
    f = DEFAULT_FLOOR_HALF

    # Floor mesh at z=0
    floor_verts = np.array([
        [-f, -f, 0], [f, -f, 0], [f, f, 0],
        [-f, -f, 0], [f, f, 0], [-f, f, 0],
    ], dtype=np.float32)
    scene_visuals.Mesh(vertices=floor_verts, color=FLOOR_COLOR, parent=view.scene)

    # Minor grid (0.25m spacing, skip where major lines go)
    minor_pts: list[list[list[float]]] = []
    n_minor = int(f / DEFAULT_GRID_MINOR)
    major_step = max(1, int(round(DEFAULT_GRID_MAJOR / DEFAULT_GRID_MINOR)))
    for i in range(-n_minor, n_minor + 1):
        if i % major_step == 0:
            continue
        offset = i * DEFAULT_GRID_MINOR
        minor_pts.append([[-f, offset, FLOOR_Z_MINOR], [f, offset, FLOOR_Z_MINOR]])
        minor_pts.append([[offset, -f, FLOOR_Z_MINOR], [offset, f, FLOOR_Z_MINOR]])
    if minor_pts:
        scene_visuals.Line(
            pos=np.array(minor_pts, dtype=np.float32).reshape(-1, 3),
            color=GRID_MINOR, connect="segments", method="gl",
            parent=view.scene,
        )

    # Major grid (1.0m spacing, thicker and brighter)
    major_pts: list[list[list[float]]] = []
    n_major = int(f / DEFAULT_GRID_MAJOR)
    for i in range(-n_major, n_major + 1):
        offset = i * DEFAULT_GRID_MAJOR
        major_pts.append([[-f, offset, FLOOR_Z_MAJOR], [f, offset, FLOOR_Z_MAJOR]])
        major_pts.append([[offset, -f, FLOOR_Z_MAJOR], [offset, f, FLOOR_Z_MAJOR]])
    if major_pts:
        scene_visuals.Line(
            pos=np.array(major_pts, dtype=np.float32).reshape(-1, 3),
            color=GRID_MAJOR, connect="segments", method="gl",
            parent=view.scene,
        )

    # Floor border
    border_pts = np.array([
        [-f, -f, FLOOR_Z_BORDER], [ f, -f, FLOOR_Z_BORDER],
        [ f, -f, FLOOR_Z_BORDER], [ f,  f, FLOOR_Z_BORDER],
        [ f,  f, FLOOR_Z_BORDER], [-f,  f, FLOOR_Z_BORDER],
        [-f,  f, FLOOR_Z_BORDER], [-f, -f, FLOOR_Z_BORDER],
    ], dtype=np.float32)
    scene_visuals.Line(
        pos=border_pts, color=FLOOR_BORDER,
        connect="segments", method="gl", parent=view.scene,
    )

    # XYZ axes (per-vertex colors: X=red, Y=green, Z=blue)
    axis_len = DEFAULT_AXIS_LENGTH
    axis_pts = np.array([
        [0, 0, 0], [axis_len, 0, 0],
        [0, 0, 0], [0, axis_len, 0],
        [0, 0, 0], [0, 0, axis_len],
    ], dtype=np.float32)
    axis_cols = np.array([
        [*AXIS_X_RGB, 1.0], [*AXIS_X_RGB, 1.0],
        [*AXIS_Y_RGB, 1.0], [*AXIS_Y_RGB, 1.0],
        [*AXIS_Z_RGB, 1.0], [*AXIS_Z_RGB, 1.0],
    ], dtype=np.float32)
    scene_visuals.Line(
        pos=axis_pts, color=axis_cols, connect="segments",
        method="gl", parent=view.scene, width=3.0,
    )

    # Axis labels (X, Y, Z)
    label_off = axis_len + AXIS_LABEL_PAD
    for char, pos, col in [
        ("X", (label_off, 0, 0), AXIS_X_RGB),
        ("Y", (0, label_off, 0), AXIS_Y_RGB),
        ("Z", (0, 0, label_off), AXIS_Z_RGB),
    ]:
        scene_visuals.Text(
            text=char, parent=view.scene, pos=pos,
            color=(*col, 1.0), font_size=14, bold=True,
            anchor_x="center", anchor_y="center",
        )

    # Origin dot
    scene_visuals.Markers(parent=view.scene).set_data(
        pos=np.array([[0, 0, 0]], dtype=np.float32),
        face_color=np.array([list(ORIGIN_DOT)], dtype=np.float32),
        edge_color=None, size=6,
    )
```

Grid lines are raised slightly above the floor (0.5mm minor, 0.6mm major, 0.7mm border) to prevent z-fighting with the floor mesh. Minor lines at 0.25m spacing are dim; major lines at 1.0m spacing are brighter and skip positions that already have a minor line. Per-vertex colors allow each axis to be a different color (red X, green Y, blue Z). The origin dot is a small white semi-transparent sphere at (0, 0, 0).

**Marker scatter visuals — continue inside `build_vispy_scene`:**

```python
    # Marker scatter: halo (larger, low-alpha) + core (normal-size, bright edge)
    halo_scatter = scene_visuals.Markers(scaling=False)
    halo_scatter.set_data(
        pos=np.array([[0, 0, 0]], dtype=np.float32),
        face_color=np.array([[0, 0, 0, 0]], dtype=np.float32),
        edge_color=None, size=1,
    )
    view.add(halo_scatter)

    core_scatter = scene_visuals.Markers(scaling=False)
    core_scatter.set_data(
        pos=np.array([[0, 0, 0]], dtype=np.float32),
        face_color=np.array([[0, 0, 0, 0]], dtype=np.float32),
        edge_color=None, size=1,
    )
    view.add(core_scatter)
```

Two `Markers` visuals: `halo_scatter` for the larger, low-alpha glow behind each drone; `core_scatter` for the normal-sized marker with a bright white edge. `scaling=False` means markers stay the same pixel size regardless of camera distance. Both start with invisible placeholder data and are updated each frame when real data arrives.

---

### 4.6: build_vispy_scene — HUD overlay

Continue inside `build_vispy_scene`. The HUD uses a 2D overlay view with a `PanZoomCamera` in pixel coordinates:

```python
    # 2D overlay view for HUD, button, and legend
    overlay_view = canvas.central_widget.add_view()
    overlay_view.camera = scene.cameras.PanZoomCamera(
        rect=(0, 0, canvas.size[0], canvas.size[1]))
    overlay_view.interactive = False
    overlay_view.order = 1  # render on top of the 3D scene
```

`PanZoomCamera` with `interactive=False` creates a 2D overlay in pixel coordinates. `order = 1` ensures the HUD renders after (on top of) the 3D scene.

**Top-left status panel — continue inside `build_vispy_scene`:**

```python
    panel_w, panel_h = 480, 124
    panel_margin = 18
    panel_pad_x = 18
    title_off_y = 22
    line_spacing = 22
    line0_off_y = title_off_y + 30

    panel = scene_visuals.Rectangle(
        center=(panel_margin + panel_w / 2,
                canvas.size[1] - panel_margin - panel_h / 2),
        width=panel_w, height=panel_h,
        color=PANEL_FILL, border_color=PANEL_BORDER,
        parent=overlay_view.scene,
    )

    title_text = scene_visuals.Text(
        text="VICON /POSES VIEWER",
        pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - title_off_y),
        color=TEXT_ACCENT, font_size=11, bold=True,
        anchor_x="left", anchor_y="top",
        parent=overlay_view.scene,
    )
    conn_text = scene_visuals.Text(
        text="", pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - line0_off_y),
        color=TEXT_DIM, font_size=10,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )
    frame_text = scene_visuals.Text(
        text="", pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - line0_off_y - line_spacing),
        color=TEXT_PRIMARY, font_size=10,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )
    active_text = scene_visuals.Text(
        text="", pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - line0_off_y - 2 * line_spacing),
        color=TEXT_PRIMARY, font_size=10,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )

    # Hint text (bottom-right)
    hint_margin = 28
    hint_text = scene_visuals.Text(
        text="R reset view   |   +/- trail length   |   S rec   |   Q/Esc quit",
        pos=(canvas.size[0] - hint_margin, hint_margin),
        color=TEXT_DIM, font_size=10,
        anchor_x="right", anchor_y="bottom",
        parent=overlay_view.scene,
    )
```

The status panel shows four lines: title ("VICON /POSES VIEWER"), connection info ("/poses"), frame/rate display, and active drone count with trail duration. The hint text at the bottom-right shows keyboard shortcuts.

**Recording button and status — continue inside `build_vispy_scene`:**

```python
    # Recording button and status (bottom-left)
    rec_btn_x, rec_btn_y = 20, 55
    rec_btn_w, rec_btn_h = 105, 30
    rec_btn_rect = scene_visuals.Rectangle(
        center=(rec_btn_x + rec_btn_w / 2, rec_btn_y + rec_btn_h / 2),
        width=rec_btn_w, height=rec_btn_h,
        color=(0.18, 0.19, 0.24, 0.88),
        border_color=(0.32, 0.33, 0.40, 0.9),
        parent=overlay_view.scene,
    )
    rec_btn_text = scene_visuals.Text(
        text="●  REC",
        pos=(rec_btn_x + rec_btn_w / 2, rec_btn_y + rec_btn_h / 2),
        color=(0.50, 0.52, 0.58, 1.0), font_size=11, bold=True,
        anchor_x="center", anchor_y="center",
        parent=overlay_view.scene,
    )
    rec_status_text = scene_visuals.Text(
        text="", pos=(rec_btn_x, rec_btn_y - 18),
        color=TEXT_DIM, font_size=9,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )
```

The recording button's bounds (`rec_btn_x`, `rec_btn_y`, `rec_btn_w`, `rec_btn_h`) are stored for mouse-click hit-testing in `_on_mouse_press`.

**Legend panel (top-right) — continue inside `build_vispy_scene`:**

```python
    # Top-right legend panel
    legend_margin = 80
    legend_w = 380
    legend_pad_x = 12
    legend_title_off_y = 22
    legend_row_start_y = legend_title_off_y + 26
    legend_row_spacing = 18

    legend_panel = scene_visuals.Rectangle(
        center=(canvas.size[0] - legend_margin - legend_w / 2,
                canvas.size[1] - panel_margin - 100 / 2),
        width=legend_w, height=100,
        color=PANEL_FILL, border_color=PANEL_BORDER,
        parent=overlay_view.scene,
    )
    legend_title = scene_visuals.Text(
        text="DRONES",
        pos=(canvas.size[0] - legend_margin - legend_w + legend_pad_x,
             canvas.size[1] - panel_margin - legend_title_off_y),
        color=TEXT_ACCENT, font_size=11, bold=True,
        anchor_x="left", anchor_y="top",
        parent=overlay_view.scene,
    )

    # Pre-allocate legend rows (swatch + name text per row)
    max_legend_rows = 20
    legend_swatches = []
    legend_rows = []
    for _ in range(max_legend_rows):
        swatch = scene_visuals.Rectangle(
            center=(0, 0), width=10, height=10,
            color=(0, 0, 0, 0), border_color=None,
            parent=overlay_view.scene,
        )
        legend_swatches.append(swatch)
        row_text = scene_visuals.Text(
            text="", pos=(0, 0),
            color=TEXT_PRIMARY, font_size=9,
            anchor_x="left", anchor_y="top", face="monospace",
            parent=overlay_view.scene,
        )
        legend_rows.append(row_text)
```

Pre-allocating legend rows avoids creating/destroying visuals when drones appear or disappear. Unused rows are moved off-screen (position (-100, -100)). The legend panel dynamically resizes based on the number of visible drones.

**Return dictionary — end of `build_vispy_scene`:**

```python
    return {
        "canvas": canvas, "view": view,
        "halo_scatter": halo_scatter, "core_scatter": core_scatter,
        "trails": {}, "shadows": {}, "drops": {}, "labels": {},
        "overlay_view": overlay_view,
        "panel": panel, "title_text": title_text,
        "conn_text": conn_text, "frame_text": frame_text,
        "active_text": active_text, "hint_text": hint_text,
        "panel_w": panel_w, "panel_h": panel_h,
        "panel_margin": panel_margin, "panel_pad_x": panel_pad_x,
        "title_off_y": title_off_y, "line0_off_y": line0_off_y,
        "line_spacing": line_spacing, "hint_margin": hint_margin,
        "marker_size": DEFAULT_MARKER_SIZE,
        "rec_btn_rect": rec_btn_rect, "rec_btn_text": rec_btn_text,
        "rec_status_text": rec_status_text,
        "rec_btn_x": rec_btn_x, "rec_btn_y": rec_btn_y,
        "rec_btn_w": rec_btn_w, "rec_btn_h": rec_btn_h,
        "legend_margin": legend_margin, "legend_w": legend_w,
        "legend_pad_x": legend_pad_x,
        "legend_title_off_y": legend_title_off_y,
        "legend_row_start_y": legend_row_start_y,
        "legend_row_spacing": legend_row_spacing,
        "legend_panel": legend_panel, "legend_title": legend_title,
        "legend_swatches": legend_swatches, "legend_rows": legend_rows,
    }
```

The dictionaries `trails`, `shadows`, `drops`, and `labels` are empty — they are populated dynamically by the `DroneViewer` class as drones appear. The layout dimensions (`panel_w`, `legend_margin`, etc.) are returned so `_relayout_hud` can recompute positions on window resize.

---

### 4.7: Obstacle loading and rendering

Add these two functions after `build_vispy_scene`:

```python
def load_obstacles(csv_path: str) -> list[dict]:
    """Parse obstacles CSV. Columns: center_x, center_y, length_x, length_y, theta."""
    obstacles = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            obstacles.append({
                "center": (float(row["center_x"]), float(row["center_y"])),
                "length": (float(row["length_x"]), float(row["length_y"])),
                "theta": float(row["theta"]),
            })
    return obstacles
```

The expected CSV columns are: `center_x, center_y, length_x, length_y, theta`. `length_x` and `length_y` are the full side lengths in meters (not half-lengths). `theta` is the rotation angle around the Z axis in radians.

```python
def add_obstacles_to_scene(view, obstacles: list[dict]):
    """Render obstacles as semi-transparent boxes with wireframe edges."""
    for obs in obstacles:
        cx, cy = obs["center"]
        lx, ly = obs["length"]
        theta = obs["theta"]

        # Rotate the four corners by theta
        c, s = math.cos(theta), math.sin(theta)
        half_x, half_y = lx / 2.0, ly / 2.0
        corners_local = np.array([
            [-half_x, -half_y], [half_x, -half_y],
            [half_x, half_y], [-half_x, half_y],
        ])
        R = np.array([[c, -s], [s, c]])
        corners_world = (R @ corners_local.T).T + np.array([cx, cy])

        # Bottom face (z=0) and top face (z=OBSTACLE_HEIGHT)
        bottom = np.array([
            [corners_world[i][0], corners_world[i][1], 0.0]
            for i in range(4)
        ], dtype=np.float32)
        top = bottom.copy()
        top[:, 2] = OBSTACLE_HEIGHT

        verts = np.vstack([bottom, top])
        faces = np.array([
            [0, 1, 2], [0, 2, 3],   # bottom
            [4, 5, 6], [4, 6, 7],   # top
            [0, 1, 5], [0, 5, 4],   # front
            [1, 2, 6], [1, 6, 5],   # right
            [2, 3, 7], [2, 7, 6],   # back
            [3, 0, 4], [3, 4, 7],   # left
        ], dtype=np.uint32)
        scene_visuals.Mesh(
            vertices=verts, faces=faces,
            color=OBSTACLE_COLOR, parent=view.scene,
        )

        # Wireframe edges (12 edges of the box)
        edge_pts = np.array([
            bottom[0], bottom[1], bottom[1], bottom[2],
            bottom[2], bottom[3], bottom[3], bottom[0],
            top[0], top[1], top[1], top[2],
            top[2], top[3], top[3], top[0],
            bottom[0], top[0], bottom[1], top[1],
            bottom[2], top[2], bottom[3], top[3],
        ], dtype=np.float32)
        scene_visuals.Line(
            pos=edge_pts, color=OBSTACLE_EDGE_COLOR,
            connect="segments", method="gl", parent=view.scene, width=1.0,
        )
```

Each obstacle is rendered as two vispy visuals: a semi-transparent `Mesh` for the faces (15% alpha dark red) and a `Line` for the wireframe edges (40% alpha brighter red). The box sits on the floor (bottom at z=0, top at z=0.4m). The coordinate transform rotates the four corners by `theta` around the Z axis, then translates by `(cx, cy)`. Obstacles are static — loaded once at startup and never updated.

---

### 4.8: PoseSubscriber ROS2 node

Add after the obstacle functions:

```python
class PoseSubscriber(Node):
    """Subscribe to /poses and feed the DataBridge."""

    def __init__(self, bridge: DataBridge):
        super().__init__("drone_viewer_pose_subscriber")
        self._bridge = bridge
        self._frame_count = 0
        self._last_msg_time: float | None = None
        self._frame_rate = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(NamedPoseArray, "/poses", self._on_poses, qos)
        self.get_logger().info("Listening on /poses")
```

The QoS profile must match the publisher's QoS (`motion_capture_tracking_node` or `fake_mocap`). Mismatched QoS causes messages to be silently dropped by the ROS2 middleware.

```python
    def _on_poses(self, msg: NamedPoseArray):
        """Callback for each /poses message."""
        drones: dict[str, tuple[float, float, float]] = {}
        for named_pose in msg.poses:
            p = named_pose.pose.position
            if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
                continue
            # /poses is already in meters — no unit conversion needed
            drones[named_pose.name] = (float(p.x), float(p.y), float(p.z))

        # Exponential moving average of frame rate
        now = time.time()
        if self._last_msg_time is not None:
            dt = now - self._last_msg_time
            if dt > 0:
                instant = 1.0 / max(dt, 1e-6)
                self._frame_rate = 0.9 * self._frame_rate + 0.1 * instant
        self._last_msg_time = now

        self._frame_count += 1
        self._bridge.update(drones, self._frame_count, self._frame_rate)
```

The callback filters out NaN positions (which can occur with single-marker Vicon when the marker is occluded). The frame rate is computed with an exponential moving average (90% old, 10% new) to smooth out jitter from network timing variations. `/poses` positions are already in meters — no unit conversion is applied.

---

### 4.9: DroneViewer — __init__ and HUD layout

Add after `PoseSubscriber`. This class owns all vispy state and rendering logic:

```python
class DroneViewer:
    """Owns the vispy scene, ROS spin timer, HUD, legend, and per-frame updates."""

    def __init__(self, bridge: DataBridge, subscriber: PoseSubscriber,
                 logger: TrajectoryLogger | None = None):
        self.bridge = bridge
        self._subscriber = subscriber
        self._logger = logger

        # Build the 3D scene and unpack the returned dictionary
        s = build_vispy_scene()
        self.canvas        = s["canvas"]
        self._view         = s["view"]
        self.halo_scatter  = s["halo_scatter"]
        self.core_scatter  = s["core_scatter"]
        self._trails       = s["trails"]
        self._shadows      = s["shadows"]
        self._drops        = s["drops"]
        self._labels       = s["labels"]
        self._overlay_view = s["overlay_view"]
        self._panel        = s["panel"]
        self._title_text   = s["title_text"]
        self._conn_text    = s["conn_text"]
        self._frame_text   = s["frame_text"]
        self._active_text  = s["active_text"]
        self._hint_text    = s["hint_text"]
        self._panel_w      = s["panel_w"]
        self._panel_h      = s["panel_h"]
        self._panel_margin = s["panel_margin"]
        self._panel_pad_x  = s["panel_pad_x"]
        self._title_off_y  = s["title_off_y"]
        self._line0_off_y  = s["line0_off_y"]
        self._line_spacing = s["line_spacing"]
        self._hint_margin  = s["hint_margin"]
        self._marker_size  = s["marker_size"]

        self._rec_btn_rect    = s["rec_btn_rect"]
        self._rec_btn_text    = s["rec_btn_text"]
        self._rec_status_text = s["rec_status_text"]
        self._rec_btn_x       = s["rec_btn_x"]
        self._rec_btn_y       = s["rec_btn_y"]
        self._rec_btn_w       = s["rec_btn_w"]
        self._rec_btn_h       = s["rec_btn_h"]

        self._legend_margin      = s["legend_margin"]
        self._legend_w           = s["legend_w"]
        self._legend_pad_x       = s["legend_pad_x"]
        self._legend_title_off_y = s["legend_title_off_y"]
        self._legend_row_start_y = s["legend_row_start_y"]
        self._legend_row_spacing = s["legend_row_spacing"]
        self._legend_panel       = s["legend_panel"]
        self._legend_title       = s["legend_title"]
        self._legend_swatches    = s["legend_swatches"]
        self._legend_rows        = s["legend_rows"]

        self.known_names: set[str] = set()

        # Connect events
        self.canvas.events.resize.connect(self._relayout_hud)
        self.canvas.events.key_press.connect(self._on_key)
        self.canvas.events.mouse_press.connect(self._on_mouse_press)
        self._relayout_hud()  # initial layout

        # ROS2 executor (processes callbacks cooperatively on the main thread)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(subscriber)

        # Timers: display at 30Hz, ROS spin at 100Hz
        self._timer = app.Timer(
            interval=0.033, connect=self._update_display, start=True)
        self._ros_timer = app.Timer(
            interval=0.01, connect=self._spin_ros, start=True)
```

`SingleThreadedExecutor` processes callbacks in the calling thread. Since `_spin_ros` is called from a vispy timer on the main thread, all ROS2 callbacks also run on the main thread — no locking is needed between the ROS2 callback and the display update. The display timer fires at 33ms (~30Hz). The ROS2 spin timer fires at 10ms (~100Hz) to drain the callback queue faster than data arrives.

**HUD layout method — add inside `DroneViewer`:**

```python
    def _relayout_hud(self, *_args):
        """Reposition HUD elements when the canvas is resized."""
        w, h = self.canvas.size
        self._overlay_view.camera.rect = (0, 0, w, h)

        self._panel.center = (
            self._panel_margin + self._panel_w / 2,
            h - self._panel_margin - self._panel_h / 2,
        )

        x = self._panel_margin + self._panel_pad_x
        self._title_text.pos  = (x, h - self._panel_margin - self._title_off_y)
        self._conn_text.pos   = (x, h - self._panel_margin - self._line0_off_y)
        self._frame_text.pos  = (x, h - self._panel_margin - self._line0_off_y - self._line_spacing)
        self._active_text.pos = (x, h - self._panel_margin - self._line0_off_y - 2 * self._line_spacing)
        self._hint_text.pos   = (w - self._hint_margin, self._hint_margin)

        legend_left = w - self._legend_margin - self._legend_w
        self._legend_title.pos = (
            legend_left + self._legend_pad_x,
            h - self._panel_margin - self._legend_title_off_y,
        )
```

All HUD element positions are computed relative to the canvas dimensions. This method is called on canvas resize and once during initialization. The `overlay_view.camera.rect` is updated so the 2D coordinate system matches the new canvas size.

**Key and mouse handlers — add inside `DroneViewer`:**

```python
    def _on_key(self, event):
        """Handle key presses."""
        key = event.key
        if key in ("Q", "Escape"):
            self.canvas.close()
        elif key == "R":
            self._view.camera.reset()
            self._view.camera.center = (0, 0, 0.4)
            self._view.camera.scale_factor = 4.5
        elif key in ("+", "="):
            self.bridge.trail_duration = min(120, self.bridge.trail_duration + 5)
            print(f"Trail duration: {self.bridge.trail_duration:.0f}s")
        elif key == "-":
            self.bridge.trail_duration = max(2, self.bridge.trail_duration - 5)
            print(f"Trail duration: {self.bridge.trail_duration:.0f}s")
        elif key == "S":
            self._toggle_recording()

    def _on_mouse_press(self, event):
        """Detect clicks on the recording button."""
        x, y = event.pos[:2]
        bx, by = self._rec_btn_x, self._rec_btn_y
        bw, bh = self._rec_btn_w, self._rec_btn_h
        if bx <= x <= bx + bw and by <= y <= by + bh:
            self._toggle_recording()

    def _toggle_recording(self):
        """Start or stop CSV recording."""
        if self._logger is None:
            return
        if self._logger.is_recording:
            path = self._logger.stop()
            print(f"Recording stopped: {path} ({self._logger.frame_count} frames)")
        else:
            path = self._logger.start()
            print(f"Recording started: {path}")
```

Key bindings: Q/Escape closes the viewer. R resets the camera. +/- adjusts trail duration in 5-second steps (clamped to 2–120 seconds). S toggles CSV recording. The mouse handler uses a direct hit-test against the recording button bounds.

---

### 4.10: DroneViewer — _update_display (markers, trails, shadows, drops, labels)

Add inside `DroneViewer`. This is the main rendering method, called at 30Hz:

```python
    def _update_display(self, _event):
        """Called at 30Hz. Reads latest data and updates all visuals."""
        current, trail_data, frame_num, frame_rate, num_drones = \
            self.bridge.snapshot()
```

**Marker scatter update — continue inside `_update_display`:**

```python
        # --- Scatter (halo + core) ---
        if current:
            names = sorted(current.keys())
            positions = np.array([current[name] for name in names], dtype=np.float32)
            colors = np.array([drone_color(name) for name in names], dtype=np.float32)

            halo_colors = colors.copy()
            halo_colors[:, 3] = 0.28
            self.halo_scatter.set_data(
                pos=positions, face_color=halo_colors,
                edge_color=None, size=self._marker_size * 2.4,
            )

            edge = np.tile(np.array(EDGE_HIGHLIGHT, dtype=np.float32),
                           (len(names), 1))
            self.core_scatter.set_data(
                pos=positions, face_color=colors,
                edge_color=edge, edge_width=1.4,
                size=self._marker_size,
            )
        else:
            # Hide markers when no data is available
            invisible = np.array([[0, 0, 0]], dtype=np.float32)
            clear = np.array([[0, 0, 0, 0]], dtype=np.float32)
            self.halo_scatter.set_data(pos=invisible, face_color=clear,
                                       edge_color=None, size=1)
            self.core_scatter.set_data(pos=invisible, face_color=clear,
                                       edge_color=None, size=1)
```

The scatter plot shows current drone positions as halo+core markers. The halo is 2.4× larger with 28% alpha; the core is normal-sized with a bright white edge. When no data is available, both are hidden by moving them to the origin with zero alpha.

**Removing stale visuals — continue inside `_update_display`:**

```python
        # --- Remove visuals for drones that disappeared ---
        active_names = set(trail_data.keys())
        for name in list(self.known_names):
            if name not in active_names:
                for group in (self._trails, self._shadows,
                              self._drops, self._labels):
                    if name in group:
                        group[name].parent = None
                        del group[name]
        self.known_names.clear()
        self.known_names.update(active_names)
```

Drones that disappeared from the data stream have their trail, shadow, drop-line, and label visuals removed from the scene (setting `parent = None` detaches them from the scenegraph).

**Per-drone trail, shadow, drop-line, and label — continue inside `_update_display`:**

```python
        # --- Update per-drone visuals ---
        for name in active_names:
            pts = trail_data[name]
            if len(pts) < 2:
                continue

            base = drone_color(name)
            arr = np.array(pts, dtype=np.float32)
            n = len(arr)

            # Trail line (alpha gradient from old to new)
            tcol = trail_color_array(base[:3], n, max_alpha=0.95, gamma=1.4)
            if name not in self._trails:
                line = scene_visuals.Line(
                    pos=arr, color=tcol, connect="strip",
                    method="gl", width=2.5,
                )
                self._view.add(line)
                self._trails[name] = line
            else:
                self._trails[name].set_data(pos=arr, color=tcol)

            # Floor shadow (projection at z=FLOOR_Z_VIS)
            shadow_pts = arr.copy()
            shadow_pts[:, 2] = FLOOR_Z_VIS
            scol = shadow_color_array(n, max_alpha=0.30)
            if name not in self._shadows:
                shadow = scene_visuals.Line(
                    pos=shadow_pts, color=scol, connect="strip",
                    method="gl", width=1.4,
                )
                self._view.add(shadow)
                self._shadows[name] = shadow
            else:
                self._shadows[name].set_data(pos=shadow_pts, color=scol)

            # Drop line (vertical from drone to floor)
            if name in current:
                x, y, z = current[name]
                drop_pts = np.array(
                    [[x, y, z], [x, y, FLOOR_Z_VIS]], dtype=np.float32)
                drop_col = np.array([
                    [base[0], base[1], base[2], 0.55],   # top (drone)
                    [base[0], base[1], base[2], 0.08],   # bottom (floor)
                ], dtype=np.float32)
                if name not in self._drops:
                    drop = scene_visuals.Line(
                        pos=drop_pts, color=drop_col, connect="strip",
                        method="gl", width=1.2,
                    )
                    self._view.add(drop)
                    self._drops[name] = drop
                else:
                    self._drops[name].set_data(pos=drop_pts, color=drop_col)

                # Text label (drone name floating above the marker)
                label_pos = (x, y, z + LABEL_Z_OFFSET)
                if name in self._labels:
                    self._labels[name].pos = label_pos
                    self._labels[name].text = name
                else:
                    self._labels[name] = scene_visuals.Text(
                        text=name, parent=self._view.scene,
                        pos=label_pos, color=base[:3] + (0.95,),
                        font_size=10, bold=True,
                        anchor_x="center", anchor_y="bottom",
                    )
```

Each drone gets four visuals: a colored trail line (alpha gradient from old to new, using `connect="strip"` for a continuous polyline), a grey floor shadow (projected at z=0.001), a vertical drop-line from the drone to the floor (bright at top, near-transparent at bottom), and a text label showing the drone name floating 7cm above the marker. New visuals are created on first appearance; existing visuals are updated in-place via `set_data()` for efficiency.

---

### 4.11: DroneViewer — HUD text, legend, recording UI, and _spin_ros

Continue inside `_update_display`, after the per-drone visuals:

```python
        # --- HUD text ---
        self._conn_text.text = "/poses  ·  BEST_EFFORT  ·  meters"
        self._frame_text.text = (
            f"Frame  {frame_num:>10d}     {frame_rate:6.1f} Hz")
        self._active_text.text = (
            f"Active {num_drones:>10d}     "
            f"Trail {self.bridge.trail_duration:5.1f} s")

        # --- Legend panel ---
        w, h = self.canvas.size
        legend_left = w - self._legend_margin - self._legend_w
        active_names_sorted = sorted(current.keys()) if current else []

        for i in range(len(self._legend_swatches)):
            if i < len(active_names_sorted):
                name = active_names_sorted[i]
                x, y, z = current[name]
                row_y = (h - self._panel_margin - self._legend_row_start_y
                         - i * self._legend_row_spacing)
                swatch_x = legend_left + self._legend_pad_x + 4

                self._legend_swatches[i].center = (swatch_x, row_y - 4)
                self._legend_swatches[i].color = drone_color(name)[:3] + (0.95,)

                self._legend_rows[i].pos = (
                    legend_left + self._legend_pad_x + 14, row_y)
                self._legend_rows[i].text = (
                    f"{name:<8s} ({x:+.3f}, {y:+.3f}, {z:+.3f}) m")
                self._legend_rows[i].color = TEXT_PRIMARY
            else:
                # Hide unused rows
                self._legend_swatches[i].center = (-100, -100)
                self._legend_rows[i].text = ""

        # Resize legend panel to fit visible rows
        n = max(len(active_names_sorted), 1)
        legend_panel_h = (self._legend_row_start_y
                          + n * self._legend_row_spacing + 10)
        self._legend_panel.height = legend_panel_h
        self._legend_panel.center = (
            legend_left + self._legend_w / 2,
            h - self._panel_margin - legend_panel_h / 2,
        )

        # --- Recording UI ---
        if self._logger is not None and self._logger.is_recording:
            if current:
                self._logger.log_frame(current, frame_num)
            elapsed = time.time() - self._logger.start_time
            mins, secs = divmod(int(elapsed), 60)
            self._rec_status_text.text = (
                f"● REC  {mins:02d}:{secs:02d}  ·  "
                f"{self._logger.frame_count} frames")
            self._rec_status_text.color = (1.0, 0.35, 0.40, 1.0)
            self._rec_btn_rect.color = (0.55, 0.12, 0.15, 0.90)
            self._rec_btn_rect.border_color = (0.90, 0.25, 0.30, 0.95)
            self._rec_btn_text.text = "■  STOP"
            self._rec_btn_text.color = (1.0, 0.35, 0.40, 1.0)
        elif self._logger is not None:
            self._rec_status_text.text = ""
            self._rec_btn_rect.color = (0.18, 0.19, 0.24, 0.88)
            self._rec_btn_rect.border_color = (0.32, 0.33, 0.40, 0.9)
            self._rec_btn_text.text = "●  REC"
            self._rec_btn_text.color = (0.50, 0.52, 0.58, 1.0)

        self.canvas.update()
```

The recording button changes appearance when active: grey background becomes dark red, "● REC" changes to "■ STOP", and a status line appears showing elapsed time and frame count. The legend panel dynamically resizes based on the number of visible drones. Unused legend rows are moved off-screen at (-100, -100).

**ROS2 spin and run — add inside `DroneViewer`:**

```python
    def _spin_ros(self, _event):
        """Drain ROS2 callbacks without blocking the GUI. Called at ~100Hz."""
        for _ in range(10):
            self._executor.spin_once(timeout_sec=0.0)

    def run(self):
        """Show the canvas and enter the vispy event loop. Blocks until closed."""
        try:
            self.canvas.show()
            app.run()
        except KeyboardInterrupt:
            pass
        finally:
            if self._logger is not None and self._logger.is_recording:
                path = self._logger.stop()
                print(f"Recording stopped: {path} "
                      f"({self._logger.frame_count} frames)")
            self._executor.remove_node(self._subscriber)
```

`spin_once(timeout_sec=0.0)` is non-blocking — returns immediately if no callbacks are ready. The loop runs 10 times per timer tick (10 × 100Hz = effectively 1000Hz polling), ensuring the callback queue is always drained faster than data arrives. `canvas.show()` makes the window visible, and `app.run()` enters the vispy event loop — this blocks until the canvas is closed. The `finally` block ensures any in-progress recording is stopped and the CSV file is properly closed.

---

### 4.12: main function

Add at the bottom of the file (outside any class):

```python
def main():
    parser = argparse.ArgumentParser(description="Drone 3D Viewer")
    parser.add_argument("--output-dir", default="recordings",
                        help="CSV recording output directory")
    parser.add_argument("--trail", type=float, default=30.0,
                        help="Trail duration in seconds (default: 30)")
    parser.add_argument("--obstacles", default="",
                        help="Path to obstacles CSV file")

    # ROS2 may append --ros-args. Keep argparse focused on this script's args.
    argv = sys.argv[1:]
    ros_args = []
    clean_args = []
    for i, arg in enumerate(argv):
        if arg == "--ros-args":
            ros_args = argv[i:]
            break
        clean_args.append(arg)

    args = parser.parse_args(clean_args)

    rclpy.init(args=ros_args if ros_args else None)
    bridge = DataBridge(trail_duration=args.trail)
    subscriber = PoseSubscriber(bridge)
    logger = TrajectoryLogger(output_dir=args.output_dir)
    viewer = DroneViewer(bridge, subscriber, logger)

    # Load obstacles if provided
    if args.obstacles:
        if os.path.exists(args.obstacles):
            obstacles = load_obstacles(args.obstacles)
            add_obstacles_to_scene(viewer._view, obstacles)
            print(f"Loaded {len(obstacles)} obstacles from {args.obstacles}")
        else:
            print(f"Obstacles file not found: {args.obstacles}", file=sys.stderr)

    try:
        viewer.run()
    finally:
        subscriber.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

**ROS2 argument filtering:** `ros2 run` passes `--ros-args ...` to the script. The argparse library does not recognize these arguments. The loop separates ROS arguments from script arguments before parsing so argparse only sees `--output-dir`, `--trail`, and `--obstacles`.

Obstacles are loaded after the scene is built but before the event loop starts. They are static — never updated during the session. `rclpy.try_shutdown()` safely handles the case where ROS2 was not initialized (e.g., if the script exits early due to an error).

---

## Section 5: Write the Launch File

Create `launch/viewer.launch.py`:

```bash
touch launch/viewer.launch.py
```

This launch file supports two modes controlled by the `mocap` argument, plus an optional viewer launch.

```python
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vicon_viewer')
    crazyflies_path = os.path.join(pkg_dir, 'config', 'crazyflies.yaml')

    # Fake mocap node — runs when mocap:=false (default)
    fake_mocap = Node(
        package='vicon_viewer',
        executable='fake_mocap',
        name='fake_mocap',
        output='screen',
        arguments=['--config', crazyflies_path,
                   '--height', '0.3'],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('mocap'), "' == 'false'"]
        )),
    )

    # Viewer node — runs when viewer:=true
    viewer_node = Node(
        package='vicon_viewer',
        executable='drone_viewer',
        name='drone_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mocap', default_value='false',
                              description="Use real Vicon (true) or fake (false)"),
        DeclareLaunchArgument('viewer', default_value='false',
                              description="Launch the drone viewer"),
        fake_mocap,
        viewer_node,
    ])
```

When `mocap:=false` (the default), only the `fake_mocap` node launches — no Vicon hardware needed. When `mocap:=true`, the CS2 mocap pipeline must be launched separately (documented in the testing section); without it, `mocap:=true viewer:=true` starts a viewer with no `/poses` data. The `viewer:=true` flag optionally launches the viewer alongside the data source.

---

## Section 6: Register Entry Points

Edit `setup.py`. Open the file at `vicon_viewer/setup.py` and replace its contents:

```python
from setuptools import setup
from glob import glob

package_name = 'vicon_viewer'

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
    description='Example 4: Vicon streaming and visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_viewer = vicon_viewer.drone_viewer:main',
            'fake_mocap = vicon_viewer.fake_mocap:main',
        ],
    },
)
```

Two entry points: `drone_viewer` for the vispy viewer and `fake_mocap` for the test data source. `glob('config/*')` installs `crazyflies.yaml` to the package's share directory. `glob('launch/*.py')` installs the launch file.

---

## Section 7: Build and Test

### 7.1: Build

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
colcon build --symlink-install
```

### 7.2: Phase 1 — Fake mocap (no hardware)

The recommended testing method uses separate terminals so the output of each node is visible. The launch file is also available as a convenience shortcut.

**Separate terminals (recommended):**

Terminal 1 — start the fake mocap publisher:
```bash
source install/setup.bash
ros2 run vicon_viewer fake_mocap
```

Terminal 2 — start the viewer:
```bash
source install/setup.bash
ros2 run vicon_viewer drone_viewer --trail 30
```

**Launch file (convenience):**
```bash
source install/setup.bash
ros2 launch vicon_viewer viewer.launch.py mocap:=false viewer:=true
```

**Expected behavior:** The vispy window opens showing a 7×7m floor with grid lines, XYZ axes, and HUD elements. Three colored markers (cf1, cf2, cf3) move in Lissajous patterns with fading trails, floor shadows, drop-lines, and position labels. The HUD shows frame count, FPS, drone count, and trail duration.

**Test the interactive features:**
- Press `+`/`-` to change trail duration (observe trails growing/shrinking)
- Press `S` to start CSV recording (the REC button turns red). Press `S` again to stop. Check the `recordings/` directory for the CSV file.
- Press `R` to reset the camera view
- Click the REC button with the mouse (same effect as pressing S)
- Press `Q` or `Escape` to close the viewer

**Test obstacles:** Create a sample obstacles CSV file at `$CRAZYFLIE_TUTORIAL/obstacles_test.csv`:

```csv
center_x,center_y,length_x,length_y,theta
1.0,1.0,0.3,0.3,0.0
-1.0,-0.5,0.5,0.2,0.785
```

Run the viewer with the obstacles file:
```bash
ros2 run vicon_viewer drone_viewer --obstacles $CRAZYFLIE_TUTORIAL/obstacles_test.csv
```

Two semi-transparent dark-red boxes should appear: one at (1.0, 1.0) oriented at 0°, and one at (-1.0, -0.5) rotated 45° (0.785 rad).

### 7.3: Phase 2 — Real Vicon (hardware required)

This phase uses the real Vicon mocap pipeline. The CS2 launch file is used with the custom configuration files written in Section 2 — the `crazyflies_yaml_file` and `motion_capture_yaml_file` arguments override the CS2 defaults (which are configured for OptiTrack, not Vicon).

**Separate terminals (recommended):**

Terminal 1 — launch the CS2 mocap pipeline with the custom configs:
```bash
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib mocap:=True gui:=False rviz:=False \
  crazyflies_yaml_file:=$CRAZYFLIE_TUTORIAL/tutorial_ws/src/vicon_viewer/config/crazyflies.yaml \
  motion_capture_yaml_file:=$CRAZYFLIE_TUTORIAL/tutorial_ws/src/vicon_viewer/config/motion_capture.yaml
```

The `crazyflie_server` will fail to connect (no real drones or Crazyradio) — this is expected. The viewer only needs the `motion_capture_tracking_node`, which publishes `/poses` from Vicon data.

Terminal 2 — start the viewer:
```bash
source install/setup.bash
ros2 run vicon_viewer drone_viewer
```

**Verification:** Place a drone with a reflective marker in the Vicon capture volume. The viewer should show the marker at its physical position. Confirm the position matches the physical placement before any flight attempt. If no marker appears, verify the mocap pipeline using the manual checks described in the mocap prerequisites (Part 0, Section 2 Step 4): `ros2 topic echo /poses`.

---

## Key Concepts

- **`NamedPoseArray` message format** — how mocap data flows from Vicon through `motion_capture_tracking_node` to any subscriber on `/poses`
- **BEST_EFFORT QoS** — why high-rate sensor data uses unreliable delivery (a fresh reading is preferred over a delayed one)
- **vispy 3D scene** — Canvas, TurntableCamera, Mesh (floor), Line (grid, axes, trails, shadows, drop-lines, wireframe), Markers (scatter), Text (labels, HUD), Rectangle (panels, buttons)
- **Thread safety via cooperative single-threading** — `SingleThreadedExecutor` + vispy timers on the main thread avoids needing locks between ROS2 callbacks and rendering (the `DataBridge` lock is present as a safeguard but not strictly required in this design)
- **CSV recording** — timestamped position data for post-flight analysis, toggled via keyboard or mouse
- **Obstacle rendering** — parsing CSV data, rendering 3D boxes with transparent faces and wireframe edges
- **Testing without hardware** — `fake_mocap` as a development tool

---

## Potential Issues

- **vispy window opens but is black** — GPU driver issue. Run `glxinfo | grep "OpenGL renderer"` to verify hardware acceleration. In a VM without GPU passthrough, vispy may not function.
- **No markers appear** — verify `/poses` is being published: `ros2 topic echo /poses`. For `fake_mocap`, check that the `--config` path is correct and the YAML file is valid.
- **FPS is very low (< 10)** — the system may be CPU-bound. Reduce trail duration (`--trail 5`), close other applications, or check GPU acceleration status.
- **Recording button does not respond to clicks** — vispy mouse coordinates use bottom-left origin. The coordinate conversion in `_on_mouse_press` may need adjustment for different display configurations.
- **Obstacles do not appear** — check the CSV path and column names. Columns must be exactly `center_x, center_y, length_x, length_y, theta`.
- **`ModuleNotFoundError: No module named 'vispy'`** — vispy was not installed. See prerequisites Section 1(b): `python3 -m pip install --user vispy`.
