# Example 2 — Sim Trajectory Flight

In Example 1, the drone hovered at a fixed position. This example makes it fly a **square trajectory** — four straight segments with sharp corners. This introduces dynamic setpoints (changing position each frame), velocity feedforward (telling the drone its desired speed, not just position), and configurable trajectory parameters.

This example assumes having completed Example 1. Reused patterns (`RateController`, `FlightSession`, `publish_full_state`, the config and launch file structure) get brief recaps. The new concepts are explained in full detail.

---

## At a glance

| Item | Value |
|---|---|
| Goal | Fly a square trajectory with velocity feedforward |
| Requires hardware? | No |
| Main package | `sim_trajectory` |
| Main script | `trajectory_flight.py` |
| New concepts | Dynamic setpoints, velocity feedforward, trajectory decomposition, PID vs Mellinger |
| Expected result | Drone takes off, flies a 1m square at 0.3 m/s, lands |

---

## Contents

1. [Files Created](#files-created)
2. [Section 1: Create the Package](#section-1-create-the-package)
3. [Section 2: Write the Configuration File](#section-2-write-the-configuration-file)
4. [Section 3: Write the Launch File](#section-3-write-the-launch-file)
5. [Section 4: Write the Flight Script](#section-4-write-the-flight-script)
6. [Section 5: Register the Entry Point](#section-5-register-the-entry-point)
7. [Section 6: Build and Test](#section-6-build-and-test)
8. [Section 7: How It Works — Velocity Feedforward and Controller Choice](#section-7-how-it-works-velocity-feedforward-and-controller-choice)
9. [Verification Checklist](#verification-checklist)
10. [Potential Issues](#potential-issues)

---

## Files Created

```
tutorial_ws/src/sim_trajectory/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── crazyflies.yaml
├── launch/
│   └── sim_trajectory.launch.py
└── sim_trajectory/
    ├── __init__.py
    └── trajectory_flight.py
```

---

## Section 1: Create the Package

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws/src
ros2 pkg create sim_trajectory --build-type ament_python \
    --dependencies rclpy crazyflie_interfaces geometry_msgs std_srvs builtin_interfaces
cd sim_trajectory
mkdir -p config launch
```

Same flags as Example 1: `ament_python` for Python, the same five ROS2 dependencies (including `builtin_interfaces` for `Duration`).

---

## Section 2: Write the Configuration File

Create `config/crazyflies.yaml`:

```bash
touch config/crazyflies.yaml
```

This is **identical** to Example 1's config — same single drone `cf1` on `sim://cf1` with Mellinger controller and Kalman estimator.

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: sim://cf1
    initial_position: [0.0, 0.0, 0.0]
    type: cf21

robot_types:
  cf21:
    motion_capture:
      enabled: false
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
      controller: 2
    locSrv:
      extPosStdDev: 1e-3
      extQuatStdDev: 0.5e-1
  firmware_logging:
    enabled: false
```

Key fields (full explanation in Example 1 Section 2):
- `uri: sim://cf1` — simulation backend
- `enHighLevel: 1` — required for takeoff/land services
- `estimator: 2` — Extended Kalman Filter
- `controller: 2` — Mellinger controller

---

## Section 3: Write the Launch File

Create `launch/sim_trajectory.launch.py`:

```bash
touch launch/sim_trajectory.launch.py
```

This is the **same pattern** as Example 1's launch file — loads `crazyflies.yaml` and CS2's `server.yaml`, launches the sim server and optionally RViz. Only the package name and paths differ.

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
    # Paths
    pkg_dir = get_package_share_directory('sim_trajectory')
    crazyflies_path = os.path.join(pkg_dir, 'config', 'crazyflies.yaml')
    cs2_share = get_package_share_directory('crazyflie')
    server_yaml_path = os.path.join(cs2_share, 'config', 'server.yaml')
    urdf_path = os.path.join(cs2_share, 'urdf', 'crazyflie_description.urdf')

    # Load and merge configs
    with open(crazyflies_path) as f:
        crazyflies = yaml.safe_load(f)
    with open(server_yaml_path) as f:
        server_cfg = yaml.safe_load(f)
    with open(urdf_path) as f:
        robot_desc = f.read()

    server_params = [crazyflies]
    server_params.append(server_cfg['/crazyflie_server']['ros__parameters'])
    server_params[1]['robot_description'] = robot_desc

    # Nodes
    sim_server = Node(
        package='crazyflie_sim',
        executable='crazyflie_server',
        name='crazyflie_server',
        output='screen',
        emulate_tty=True,
        parameters=server_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(cs2_share, 'config', 'config.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        sim_server,
        rviz_node,
    ])
```

---

## Section 4: Write the Flight Script

Create `sim_trajectory/trajectory_flight.py`:

```bash
touch sim_trajectory/trajectory_flight.py
```

### 4.1: Shebang, docstring, and imports

```python
#!/usr/bin/env python3
"""
Simulated trajectory flight for one Crazyflie.

Flies a square trajectory using streaming cmd_full_state setpoints
with velocity feedforward. The drone takes off, hovers to stabilize,
flies one loop of a square, returns to the start, and lands.
"""

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import FullState
from builtin_interfaces.msg import Duration
from dataclasses import dataclass
import time
import math
```

The new import vs Example 1: `from dataclasses import dataclass` — Python's dataclass decorator. Generates `__init__`, `__repr__`, and `__eq__` automatically from type-annotated fields. It is used for the `Setpoint` class.

### 4.2: Setpoint dataclass

In Example 1, `publish_full_state(x, y, z, yaw)` was called directly. That works for a static hover but becomes cumbersome when each frame has different position, velocity, and yaw values. The `Setpoint` dataclass bundles all the fields into a single object. Default values of `0.0` for velocity, acceleration, and yaw mean only the fields that differ from a static hover need to be specified.

```python
@dataclass
class Setpoint:
    """Full-state setpoint for the Crazyflie controller.

    Position is required. Velocity, acceleration, and yaw default to
    zero — typical for hover / transition phases. Set them explicitly
    when feedforward improves tracking (e.g., tangent velocity
    during a square, yaw tracking during a circle).
    """
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    yaw: float = 0.0
```

`@dataclass` — one decorator replaces ~40 lines of boilerplate. Fields without defaults (`x`, `y`, `z`) are required. Fields with defaults (`vx=0.0`, etc.) are optional. Usage examples:
- Hover: `Setpoint(x=0.0, y=0.0, z=0.5)` — only position
- Moving right: `Setpoint(x=1.0, y=0.0, z=0.5, vx=0.3)` — position + velocity

### 4.3: RateController class (reused from Example 1)

Phase-locked rate limiter with skip-on-overrun. `start()` records the anchor time. `sleep()` advances by one period and sleeps the remaining time. Full explanation in Example 1 Section 4.2.

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

### 4.4: FlightSession class (reused from Example 1)

Context manager that guarantees landing on any exit. Full explanation in Example 1 Section 4.3.

```python
class FlightSession:
    """Context manager that lands the drone on any exit path."""
    def __init__(self, node, drone: str, land_height: float = 0.0,
                 land_duration: float = 2.5):
        self.node = node
        self.drone = drone
        self.land_height = land_height
        self.land_duration = land_duration
        self._airborne = False

    def takeoff(self, height: float, duration: float):
        self.node.takeoff(height, duration)
        self._airborne = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is KeyboardInterrupt:
            print("\n\nFlight interrupted by user — landing...")
        elif exc_type is not None:
            print(f"\n\nFlight error ({exc_type.__name__}: {exc_val}) — "
                  "landing...")
        if self._airborne:
            try:
                self.node.land(self.land_duration)
            except Exception as e:
                print(f"  WARNING: land command failed: {e}")
        return False
```

### 4.5: TrajectoryFlight node — __init__

```python
class TrajectoryFlight(Node):

    def __init__(self):
        super().__init__('trajectory_flight')

        # Drone identity
        self.drone = 'cf1'

        # Flight parameters
        self.flight_height = 0.5          # meters
        self.dt = 1.0 / 30.0              # 30 Hz control rate

        # Trajectory parameters
        self.square_side = 1.0            # length of each edge (meters)
        self.speed = 0.3                  # speed along each edge (m/s)
        self.num_loops = 1                # laps around the square

        # Timing (derived from trajectory params)
        self.segment_duration = self.square_side / self.speed
        self.trajectory_duration = 4 * self.segment_duration * self.num_loops
        self.takeoff_duration = 2.5
        self.stabilize_duration = 2.0     # hover before starting trajectory
        self.land_duration = 2.5
```

The trajectory parameters are designed to be changed. Try `square_side = 0.5` for a smaller square, `speed = 0.1` for slow precise flight, or `num_loops = 2` for two laps. The derived timing:
- `segment_duration = side / speed` — for 1.0m at 0.3 m/s: 3.33 seconds per edge
- `trajectory_duration = 4 × segment_duration × num_loops` — for 1 loop: 13.33 seconds total

```python
        # Service clients
        self.takeoff_cli = self.create_client(
            Takeoff, f'/{self.drone}/takeoff')
        self.land_cli = self.create_client(
            Land, f'/{self.drone}/land')
        self.notify_cli = self.create_client(
            NotifySetpointsStop, f'/{self.drone}/notify_setpoints_stop')

        # Publisher
        self.fullstate_pub = self.create_publisher(
            FullState, f'/{self.drone}/cmd_full_state', 1)

        # Wait for services
        self.get_logger().info('Waiting for Crazyflie services...')
        for cli in [self.takeoff_cli, self.land_cli, self.notify_cli]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'Waiting for {cli.srv_name}...')
        self.get_logger().info('All services available!')
```

Identical pattern to Example 1.

### 4.6: Service methods and the expanded publish_full_state

The service methods are the same pattern used in Example 1. The full code is provided here so this example is self-contained. Add these methods inside the `TrajectoryFlight` class:

```python
    def takeoff(self, height: float, duration: float):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = height
        req.duration = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        future = self.takeoff_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info(f'Takeoff to {height}m')

    def land(self, duration: float):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        future = self.land_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info('Landing')

    def notify_setpoints_stop(self):
        req = NotifySetpointsStop.Request()
        req.group_mask = 0
        req.remain_valid_millisecs = 100
        future = self.notify_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
```

`takeoff()` and `land()` construct service requests with the target height and maneuver duration, send them asynchronously, and block until the response arrives. `notify_setpoints_stop()` signals the firmware to switch from LLC streaming mode back to HLC mode. `group_mask = 0` targets all drone groups; `remain_valid_millisecs = 100` provides a 100 ms grace window for the last setpoint before the switch. This must be called after the last streaming setpoint and before any HLC command like `land()`.

The `publish_full_state` method expands to accept optional velocity and acceleration:

```python
    def publish_full_state(self, x: float, y: float, z: float,
                           vx: float = 0.0, vy: float = 0.0,
                           vz: float = 0.0,
                           ax: float = 0.0, ay: float = 0.0,
                           az: float = 0.0,
                           yaw: float = 0.0):
        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

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

        self.fullstate_pub.publish(msg)
```

All velocity and acceleration parameters default to `0.0` — Example 1 hover calls (`publish_full_state(x, y, z)`) still work unchanged. Only trajectory calls pass non-zero velocity.

**A note on design choices:** There are several ways to structure setpoint publishing. One approach passes a `Setpoint` object directly to a wrapper method (`publish_setpoint(sp)` — cleaner call sites but introduces a dataclass in Example 1 where it is unnecessary). Another keeps `publish_full_state` with plain float args and unpack `Setpoint` at the call site (`publish_full_state(sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz, sp.yaw)` — clear data flow but verbose). This tutorial uses the plain-float-args approach with default values: one method, grows naturally as requirements grow, consistent across all six examples. The `Setpoint` dataclass serves as a clean return type for trajectory computation methods.

### 4.7: Trajectory computation — the square

This is the core new concept. A square has four edges, numbered to match the code's `seg_idx`. Starting from the origin (bottom-left corner), the drone flies Segment 0: right (+X) along the bottom edge, Segment 1: up (+Y) along the right edge, Segment 2: left (-X) along the top edge, Segment 3: down (-Y) along the left edge, and returns to the start.

```
         Segment 2 (-X)
      ┌───────────────────┐
      │                   │
Seg 3 │                   │ Seg 1
(-Y)  │                   │ (+Y)
      │                   │
      └───────────────────┘
      Start → Segment 0 (+X)
```

```python
    def square_setpoint(self, elapsed: float):
        """Compute the setpoint for a square trajectory at time t.

        Args:
            elapsed: Time in seconds since the start of the trajectory.

        Returns:
            Setpoint at the given elapsed time. Returns None if elapsed
            is outside the trajectory duration.
        """
        if elapsed < 0 or elapsed > self.trajectory_duration:
            return None

        # Which loop and segment are we in?
        loop_time = self.trajectory_duration / self.num_loops
        loop_elapsed = elapsed % loop_time
        seg_idx = int(loop_elapsed / self.segment_duration)
        seg_t = loop_elapsed - seg_idx * self.segment_duration
        frac = seg_t / self.segment_duration  # 0.0 to 1.0 within segment

        # The start corner of the square
        x0, y0 = 0.0, 0.0

        # Position and velocity for each segment
        if seg_idx == 0:    # +X (right)
            x = x0 + self.square_side * frac
            y = y0
            vx = self.speed
            vy = 0.0
        elif seg_idx == 1:  # +Y (back)
            x = x0 + self.square_side
            y = y0 + self.square_side * frac
            vx = 0.0
            vy = self.speed
        elif seg_idx == 2:  # -X (left)
            x = x0 + self.square_side * (1.0 - frac)
            y = y0 + self.square_side
            vx = -self.speed
            vy = 0.0
        else:               # -Y (forward)
            x = x0
            y = y0 + self.square_side * (1.0 - frac)
            vx = 0.0
            vy = -self.speed

        return Setpoint(x=x, y=y, z=self.flight_height, vx=vx, vy=vy)
```

**How the timing logic works:**

`if elapsed < 0 or elapsed > self.trajectory_duration: return None` — guards against calls outside the trajectory window. `main()` uses this to know when the trajectory is done.

`loop_time = self.trajectory_duration / self.num_loops` — duration of one complete lap. If doing 2 loops, each takes half the total time.

`loop_elapsed = elapsed % loop_time` — the modulo operator `%` gives the remainder after division. After one full lap (13.33s), `13.33 % 13.33 = 0.0` — the time wraps back to the start of the first segment. This "wraps" the time for multi-loop trajectories.

`seg_idx = int(loop_elapsed / self.segment_duration)` — which segment we are in. `int()` truncates: `0.0 / 3.33 = 0` (segment 0, +X), `3.5 / 3.33 = 1.05 → 1` (segment 1, +Y), `7.0 / 3.33 = 2.10 → 2` (segment 2, -X), `10.0 / 3.33 = 3.00 → 3` (segment 3, -Y).

`frac = seg_t / self.segment_duration` — fraction of the current segment completed, from `0.0` (start) to `1.0` (end).

**How each segment computes position:**

**Segment 0 (+X):** `x = x0 + side * frac` — x moves linearly from `x0` to `x0 + side`. At `frac=0.0`, x=x0. At `frac=1.0`, x=x0+side. `y` stays constant. `vx = speed, vy = 0.0` — velocity feedforward tells the drone "the desired velocity."

**Segment 1 (+Y):** x stays at the right edge. y moves from `y0` to `y0 + side`. `vx = 0.0, vy = speed`.

**Segment 2 (-X):** x moves BACK from the right edge: `x0 + side * (1.0 - frac)`. At `frac=0.0`, x = right edge. At `frac=1.0`, x = x0 (left edge). The `(1.0 - frac)` reverses the direction. `vx = -speed`.

**Segment 3 (-Y):** x is back at x0. y moves down: `y0 + side * (1.0 - frac)`. `vy = -speed`.

**Why velocity feedforward matters:**

Without feedforward (`vx=vy=0.0`), the controller only sees position error. When the setpoint jumps ahead, the controller thinks "I need to move" and accelerates reactively. By the time it reaches the target, the setpoint has already moved further. The drone is always catching up. Result: the flown path is a **rounded shape inside** the intended square — corners are cut, edges lag behind.

With feedforward (`vx=speed`, etc.), the controller knows the desired velocity. It matches that velocity directly, plus applies small corrections for any remaining position error. Result: the flown path is a **sharp, accurate square** — corners are tight, edges track precisely.

**Analogy:** Driving by only looking at GPS position vs looking at the speedometer. The GPS-only driver is always reacting late; the speedometer driver anticipates and maintains the right speed.

### 4.8: main() function

```python
def main():
    rclpy.init()
    node = TrajectoryFlight()

    with FlightSession(node, 'cf1',
                       land_duration=node.land_duration) as flight:

        # === Phase 1: Takeoff ===
        print("Taking off...")
        flight.takeoff(node.flight_height, node.takeoff_duration)
        time.sleep(node.takeoff_duration + 1.0)

        # === Phase 2: Stabilize (hover at takeoff position) ===
        print(f"Stabilizing for {node.stabilize_duration}s...")
        rate = RateController(node.dt)
        rate.start()
        stab_start = time.monotonic()
        while time.monotonic() - stab_start < node.stabilize_duration:
            node.publish_full_state(0.0, 0.0, node.flight_height)
            rate.sleep()
```

Phases 1-2 are the same as Example 1. The stabilize phase lets the drone settle at the hover height before starting the trajectory. Without this, the drone would start flying the square while still ascending.

```python
        # === Phase 3: Fly the square trajectory ===
        print(f"Flying {node.num_loops}x square "
              f"({node.square_side}m sides, {node.speed}m/s)...")
        print(f"  Segment duration: {node.segment_duration:.1f}s")
        print(f"  Total trajectory: {node.trajectory_duration:.1f}s")

        rate.start()
        traj_start = time.monotonic()
        while time.monotonic() - traj_start < node.trajectory_duration:
            elapsed = time.monotonic() - traj_start
            sp = node.square_setpoint(elapsed)
            if sp is not None:
                node.publish_full_state(
                    sp.x, sp.y, sp.z,
                    sp.vx, sp.vy, sp.vz,
                    sp.ax, sp.ay, sp.az,
                    sp.yaw)
            rate.sleep()

        if rate.overruns:
            node.get_logger().warning(
                f'{rate.overruns} loop overruns at 30Hz')
```

The trajectory loop is structurally the same as the hover loop — `while` with `RateController` at 30Hz — but each iteration calls `square_setpoint(elapsed)` to compute a DIFFERENT setpoint. The `Setpoint` is unpacked at the call site into individual float arguments for `publish_full_state`.

`elapsed = time.monotonic() - traj_start` — time since trajectory start. This is the input to the trajectory math: at `elapsed=0`, the drone is at the start corner; at `elapsed=segment_duration`, it is at the next corner.

```python
        # === Phase 4: Hover briefly at the end ===
        print("Trajectory complete. Hovering at start...")
        rate.start()
        hover_end_start = time.monotonic()
        while time.monotonic() - hover_end_start < 1.5:
            node.publish_full_state(0.0, 0.0, node.flight_height)
            rate.sleep()

        # === Phase 5: Land ===
        print("Landing...")
        node.notify_setpoints_stop()
        time.sleep(0.1)
        node.land(node.land_duration)
        time.sleep(node.land_duration + 1.0)
        flight._airborne = False

        print("Flight complete!")
        print(f"\nTrajectory summary:")
        print(f"  Square: {node.square_side}m x {node.square_side}m")
        print(f"  Speed: {node.speed} m/s")
        print(f"  Loops: {node.num_loops}")

    node.destroy_node()
    rclpy.shutdown()
```

After the trajectory, the drone hovers for 1.5s (to stabilize after the last corner), then lands. The printed summary confirms the trajectory parameters.

### 4.9: Entry point guard

```python
if __name__ == '__main__':
    main()
```

---

## Section 5: Register the Entry Point

Edit `setup.py`:

```python
from setuptools import setup
from glob import glob

package_name = 'sim_trajectory'

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
    description='Example 2: Sim trajectory flight',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_flight = sim_trajectory.trajectory_flight:main',
        ],
    },
)
```

---

## Section 6: Build and Test

### 6.1: Build

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
colcon build --symlink-install
```

### 6.2: Launch and run

**Terminal 1** — sim server + RViz:

```bash
source install/setup.bash
ros2 launch sim_trajectory sim_trajectory.launch.py rviz:=true
```

**Terminal 2** — flight script:

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run sim_trajectory trajectory_flight --ros-args -p use_sim_time:=True
```

**Expected behavior in RViz:** The drone takes off to z=0.5, hovers for 2s, then traces a 1.0m square at 0.3 m/s — four straight edges with sharp corners. After one lap, it hovers briefly and lands.

### 6.3: Experiment with parameters

Because `--symlink-install` symlinks the Python source, the file `trajectory_flight.py` can be edited and re-run without rebuilding:

- **Change `self.square_side = 0.5`** — smaller square, 1.67s per edge
- **Change `self.speed = 0.1`** — very slow, nearly perfect tracking
- **Change `self.speed = 0.5`** — faster, may see slight corner overshoot
- **Change `self.num_loops = 2`** — two laps
- **Remove velocity feedforward** — set `vx=0.0, vy=0.0` in all four segments. Observe the drone lagging behind and rounding the corners. Then restore the feedforward and see the difference.

---

## Section 7: How It Works — Velocity Feedforward and Controller Choice

### A note on controller choice: Mellinger (sim) vs PID (hardware)

In this example's `crazyflies.yaml`, the controller is set to `stabilizer.controller: 2` — the **Mellinger controller**. Mellinger is a nonlinear controller that uses differential flatness to compute force and attitude commands from the full setpoint (position, velocity, acceleration, yaw, and angular velocity). It tracks aggressive trajectories more tightly than PID.

In the hardware examples (5-6), the controller is set to `stabilizer.controller: 1` — the **PID controller**. PID is simpler, more robust to sensor noise (real Vicon data is noisier than simulation ground truth), and safer for first physical flights.

**The critical point: `cmd_full_state` works identically with both controllers.** The message format, topic, 30Hz rate, and field meanings do not change. Both controllers receive the same setpoint structure and attempt to track it. The difference is only in the internal algorithm: PID uses three independent per-axis control loops (simpler, handles noise well, softer corners); Mellinger treats the drone as a coordinated nonlinear system (tighter tracking, more sensitive to velocity noise). From your script's perspective, the interface is unchanged — the controller choice is a firmware parameter.

### The difference feedforward makes

```
Without feedforward (vx=vy=0):        With feedforward (vx=speed, vy=0):

   Intended: ┌─────┐                   Intended: ┌─────┐
             │     │                             │     │
             │     │                             │     │
             └─────┘                             └─────┘

   Actual:   ╭─────╮                   Actual:   ┌─────┐
             ╰─────╯                             │     │
             (rounded, lags)                     └─────┘
                                                 (sharp, tight)
```

### What happens inside the controller

The Mellinger controller receives each `cmd_full_state` message containing position ("where the drone should be"), velocity ("how fast you should be moving"), and acceleration ("how speed should change").

Without velocity feedforward, the controller computes position error and generates a corrective force: "the drone is 5cm behind → push forward." This is purely **reactive** — the controller only responds after an error develops.

With velocity feedforward, the controller also knows the desired velocity. It generates a force to match that velocity, plus a smaller corrective force for remaining position error. The feedforward handles the bulk of the motion; position feedback only corrects small deviations. This is **predictive** — the controller anticipates the motion before error accumulates.

### Why corners are the hardest part

At a corner, desired velocity instantly changes from `(+0.3, 0)` to `(0, +0.3)`. The drone has inertia — it cannot change direction instantly. With feedforward, the Mellinger controller knows the target velocity and computes the exact force needed to cancel the old velocity and build the new one. Without feedforward, it only sees position error and must figure out the velocity change from position feedback — slower and less accurate.

---

## Verification Checklist

- [ ] `colcon build --symlink-install` succeeds
- [ ] `ros2 launch sim_trajectory sim_trajectory.launch.py rviz:=true` starts without errors
- [ ] `ros2 run sim_trajectory trajectory_flight --ros-args -p use_sim_time:=True` completes normally
- [ ] RViz shows a recognizable square (not a rounded circle)
- [ ] The drone holds position before and after the trajectory
- [ ] Ctrl+C during the trajectory triggers emergency landing
- [ ] Changing `self.square_side` or `self.speed` and re-running produces a different square
- [ ] Removing velocity feedforward produces visibly worse tracking

---

## Potential Issues

- **Square looks rounded/overshoots corners** — velocity feedforward may be incorrect. Check that `vx` and `vy` match the segment direction and speed. Try reducing `self.speed`.
- **Drone drifts during trajectory** — the start position `(0.0, 0.0)` must match the drone's actual hover position. Small drift (a few cm) is normal.
- **Trajectory finishes early/late** — check `segment_duration = side / speed`. The printed summary should match the actual flight time.
- **`square_setpoint()` returns None during flight** — the elapsed time check is too strict. Verify `traj_start` is recorded correctly.
- **Loop runs but setpoints are not published** — `square_setpoint()` may return `None`. Add a temporary `print()` inside the method to debug.
