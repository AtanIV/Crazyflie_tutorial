# Example 3 — Sim Multidrone Control

This example controls two simulated drones simultaneously. cf1 flies a 1.5m square; cf2 flies a 1.0m-radius circle. Both take off together, transition to their start positions, fly simultaneously, hover-wait for each other, return to their initial positions, and land together.

This example assumes having completed Examples 1 and 2. Reused patterns (`RateController`, `publish_full_state`, `Setpoint`, `FlightSession`, square trajectory, config and launch structure) get brief recaps. New concepts — multi-drone config, per-drone dicts, circle trajectory, hover-wait synchronization, and topic-based multi-drone RViz config — are explained in full detail.

---

## At a glance

| Item | Value |
|---|---|
| Goal | Fly two simulated drones simultaneously (square + circle) |
| Requires hardware? | No |
| Main package | `sim_multidrone` |
| Main script | `multi_flight.py` |
| New concepts | Multi-drone config, per-drone dicts, circle trajectory, hover-wait sync, multi-drone RViz |
| Expected result | Two drones take off together, fly their trajectories, return, and land |

---

## Contents

1. [Files Created](#files-created)
2. [Section 1: Create the Package](#section-1-create-the-package)
3. [Section 2: Write the Multi-Drone Configuration](#section-2-write-the-multi-drone-configuration)
4. [Section 3: Write the RViz Config](#section-3-write-the-rviz-config)
5. [Section 4: Write the Launch File](#section-4-write-the-launch-file)
6. [Section 5: Write the Flight Script](#section-5-write-the-flight-script)
7. [Section 6: Register the Entry Point](#section-6-register-the-entry-point)
8. [Section 7: Build and Test](#section-7-build-and-test)
9. [Section 8: Architecture Recap](#section-8-architecture-recap)
10. [Verification Checklist](#verification-checklist)
11. [Potential Issues](#potential-issues)

---

## Files Created

```
tutorial_ws/src/sim_multidrone/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   ├── crazyflies.yaml
│   └── sim_multidrone.rviz
├── launch/
│   └── sim_multidrone.launch.py
└── sim_multidrone/
    ├── __init__.py
    └── multi_flight.py
```

---

## Section 1: Create the Package

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws/src
ros2 pkg create sim_multidrone --build-type ament_python \
    --dependencies rclpy crazyflie_interfaces geometry_msgs std_srvs builtin_interfaces
cd sim_multidrone
mkdir -p config launch
```

---

## Section 2: Write the Multi-Drone Configuration

Create `config/crazyflies.yaml`:

```bash
touch config/crazyflies.yaml
```

This is the first config with two drones.

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: sim://cf1
    initial_position: [0.0, 0.0, 0.0]
    type: cf21

  cf2:
    enabled: true
    uri: sim://cf2
    initial_position: [2.5, 0.0, 0.0]
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

**What is new vs Examples 1-2:**

Two drones under `robots:` — `cf1` and `cf2`. Each has its own `uri` and `initial_position`. `cf2.initial_position: [2.5, 0.0, 0.0]` places cf2 2.5 meters to the right of cf1. This separation ensures the drones do not collide and provides visual clarity in RViz. The 2.5m gap leaves clearance between cf1's 1.5m square (with a corner at the origin, spanning x,y ∈ [0, 1.5]) and cf2's 2.0m-diameter circle (centered at x=3.5).

`cf2.uri: sim://cf2` — a different sim URI. Each simulated drone needs a unique URI. The name after `sim://` is the simulator's internal identifier.

Everything under `robot_types` and `all` is identical to the earlier examples. Both drones share the same type definition and firmware parameters. On hardware, each drone would have individual firmware params (different radio addresses), but in simulation the shared definition works fine.

---

## Section 3: Write the RViz Config

### 3.1: Custom RViz config

In Examples 1-2, RViz displayed one drone using CS2's default config with a single RobotModel display. For two drones, a custom RViz config is needed with two RobotModel displays — one per drone.

Using `Description Source: Topic` (subscribing to `/{drone}/robot_description`) causes RViz to crash with two RobotModel displays. The fix: use `Description Source: File` with **separate URDF files per drone** — each with a unique robot and link name. Two `RobotModel` displays sharing the same URDF (with identical robot names) will conflict.

Create two minimal URDF files that reference the CS2 3D mesh, then create the RViz config:

```bash
cat > config/cf1.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="cf1">
  <link name="cf1">
    <origin rpy="0 0 0" xyz="0.0 0 0" />
    <visual>
        <geometry>
          <mesh filename="package://crazyflie/urdf/cf2_assembly_with_props.dae" scale="1.0 1.0 1.0"/>
        </geometry>
    </visual>
  </link>
</robot>
EOF

cat > config/cf2.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="cf2">
  <link name="cf2">
    <origin rpy="0 0 0" xyz="0.0 0 0" />
    <visual>
        <geometry>
          <mesh filename="package://crazyflie/urdf/cf2_assembly_with_props.dae" scale="1.0 1.0 1.0"/>
        </geometry>
    </visual>
  </link>
</robot>
EOF

```

> Each URDF has a unique `robot name` and `link name` matching the drone. Both reference the same CS2 3D mesh file via `package://crazyflie` — RViz resolves this using the ROS2 package path since `crazyflie` is an installed package in the CS2 workspace (sourced in `.bashrc`). Separate URDFs prevent the robot name collision that causes RViz to segfault when two `RobotModel` displays try to load the same URDF.

```bash
cat > config/sim_multidrone.rviz << EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Enabled: true
  Displays:
    - Alpha: 0.5
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        cf1:
          Value: true
        cf2:
          Value: true
        world:
          Value: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        world:
          cf1: {}
          cf2: {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description Source: File
      Description File: $CRAZYFLIE_TUTORIAL/tutorial_ws/src/sim_multidrone/config/cf1.urdf
      Enabled: true
      Links:
        All Links Enabled: true
      Mass Properties:
        Inertia: false
        Mass: false
      Name: CF1
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description Source: File
      Description File: $CRAZYFLIE_TUTORIAL/tutorial_ws/src/sim_multidrone/config/cf2.urdf
      Enabled: true
      Links:
        All Links Enabled: true
      Mass Properties:
        Inertia: false
        Mass: false
      Name: CF2
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 7
      Focal Point:
        X: 1.25
        Y: 0
        Z: 0.5
      Name: Current View
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
EOF
```

**Key sections explained:**

`TF` display — `Frames: cf1, cf2, world` tells RViz to show coordinate axes for both drones and the world frame. The `Tree: world: {cf1: {}, cf2: {}}` defines the TF tree: both drones are children of `world`. The sim server's `rviz.py` visualization plugin publishes these transforms automatically.

`RobotModel` displays — one per drone with separate URDF files. `Description Source: File` tells RViz to read the robot model from a local file. Each drone needs its own URDF with a unique `robot name` and `link name`. The paths use `$CRAZYFLIE_TUTORIAL` which expands to the absolute tutorial root path when the heredoc runs (no quotes around `EOF`). Absolute paths are required — relative paths work for one `RobotModel` display but cause RViz to crash when two displays are present.

`Views → Focal Point: [1.25, 0, 0.5]` — centers the RViz camera between the two drones (cf1 at x=0, cf2 at x=2.5; center at x=1.25; z=0.5 is the flight height).

---

## Section 4: Write the Launch File

Create `launch/sim_multidrone.launch.py`:

```bash
touch launch/sim_multidrone.launch.py
```

Same pattern as Examples 1-2, but uses the custom RViz config.

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
    pkg_dir = get_package_share_directory('sim_multidrone')
    crazyflies_path = os.path.join(pkg_dir, 'config', 'crazyflies.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'sim_multidrone.rviz')

    cs2_share = get_package_share_directory('crazyflie')
    server_yaml_path = os.path.join(cs2_share, 'config', 'server.yaml')
    urdf_path = os.path.join(cs2_share, 'urdf', 'crazyflie_description.urdf')

    with open(crazyflies_path) as f:
        crazyflies = yaml.safe_load(f)
    with open(server_yaml_path) as f:
        server_cfg = yaml.safe_load(f)
    with open(urdf_path) as f:
        robot_desc = f.read()

    server_params = [crazyflies]
    server_params.append(server_cfg['/crazyflie_server']['ros__parameters'])
    server_params[1]['robot_description'] = robot_desc

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
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        sim_server,
        rviz_node,
    ])
```

The only difference from earlier launch files: `arguments=['-d', rviz_config]` points RViz at the custom config instead of the CS2 default.

---

## Section 5: Write the Flight Script

Create `sim_multidrone/multi_flight.py`:

```bash
touch sim_multidrone/multi_flight.py
```

### 5.1: Shebang, docstring, and imports

```python
#!/usr/bin/env python3
"""
Multi-drone streaming flight using ROS2.

Two drones fly simultaneously: cf1 flies a square, cf2 flies a circle.
All phases are sequential — each phase completes for both drones
before the next phase starts. After trajectories finish, both drones
hover and wait for each other before landing together.
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

### 5.2: Reused classes — RateController and Setpoint

Copy `RateController` and `Setpoint` identically from Example 2. No changes needed.

`RateController` (full explanation in Example 1 Section 4.2):
```python
class RateController:
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

`Setpoint` (full explanation in Example 2 Section 4.2):
```python
@dataclass
class Setpoint:
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

### 5.3: FlightSession — extended for multi-drone

The single-drone `FlightSession` tracked one `_airborne` flag. For multi-drone, airborne drones are tracked in a **set** of airborne drone names so each one gets landed on exit.

```python
class FlightSession:
    """Context manager that lands all airborne drones on any exit path."""

    def __init__(self, node, drones: list[str],
                 land_duration: float = 2.5, post_land_wait: float = 2.0):
        self.node = node
        self.drones = drones
        self.land_duration = land_duration
        self.post_land_wait = post_land_wait
        self._airborne: set[str] = set()

    def takeoff(self, drone: str, height: float, duration: float):
        self.node.takeoff(drone, height, duration)
        self._airborne.add(drone)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is KeyboardInterrupt:
            print("\n\nFlight interrupted by user — landing all drones...")
        elif exc_type is not None:
            print(f"\n\nFlight error ({exc_type.__name__}: {exc_val}) — "
                  "landing all drones...")

        for drone in list(self._airborne):
            try:
                self.node.land(drone, self.land_duration)
                print(f"  {drone}: land command sent")
            except Exception as e:
                print(f"  WARNING: land command failed for {drone}: {e}")

        if self._airborne:
            time.sleep(self.post_land_wait)

        return False
```

Changes from the single-drone version:
- `drones: list[str]` — a list of drone names instead of a single string
- `self._airborne: set[str]` — a Python set tracking which drones have been issued takeoff. Using a set avoids duplicates.
- `takeoff(drone, ...)` — adds the drone to the set individually
- `__exit__` iterates over a copy of the airborne set (`list(self._airborne)`) and lands each drone. If one fails, the others still get their land commands.
- `post_land_wait` — extra wait after all land commands are issued

### 5.4: MultiDroneFlight node — __init__

```python
class MultiDroneFlight(Node):

    def __init__(self):
        super().__init__('multi_drone_flight')

        # Drone list
        self.drones = ['cf1', 'cf2']

        # Flight parameters (shared by both drones)
        self.flight_height = 0.5          # meters
        self.dt = 1.0 / 30.0              # 30 Hz

        # Trajectory parameters
        self.square_side = 1.5            # cf1's square edge length
        self.circle_radius = 1.0          # cf2's circle radius
        self.speed = 0.3                  # m/s (shared speed)

        # Timing
        self.takeoff_duration = 2.5
        self.stabilize_duration = 2.0
        self.transition_duration = 5.0    # time to reach start positions
        self.land_duration = 2.5
        self.phase_pause = 1.0            # pause between phases

        # Derived timing
        self.square_segment_duration = self.square_side / self.speed
        self.square_duration = 4 * self.square_segment_duration
        self.circle_duration = (2 * math.pi * self.circle_radius) / self.speed
```

The derived timing shows an important relationship:
- Square: 4 × 1.5m / 0.3 = **20.0 seconds**
- Circle: 2π × 1.0m / 0.3 ≈ **20.9 seconds**

The circle is slightly longer. This is intentional — it demonstrates the hover-wait pattern: cf1 (square) finishes first and hovers at its end position while cf2 completes its last ~0.9 seconds of the circle.

```python
        # Service clients — one per drone, stored in dictionaries
        self.takeoff_clients = {}
        self.land_clients = {}
        self.notify_clients = {}
        for drone in self.drones:
            self.takeoff_clients[drone] = self.create_client(
                Takeoff, f'/{drone}/takeoff')
            self.land_clients[drone] = self.create_client(
                Land, f'/{drone}/land')
            self.notify_clients[drone] = self.create_client(
                NotifySetpointsStop, f'/{drone}/notify_setpoints_stop')

        # Publishers — one per drone
        self.fullstate_pubs = {}
        for drone in self.drones:
            self.fullstate_pubs[drone] = self.create_publisher(
                FullState, f'/{drone}/cmd_full_state', 1)
```

Instead of `self.takeoff_cli` (single client), we have `self.takeoff_clients` — a dictionary mapping drone name → client. `self.takeoff_clients['cf1']` gives cf1's takeoff client. This pattern scales: to add a third drone, just add `'cf3'` to `self.drones` and the `for` loop creates all the clients automatically.

```python
        # Initial positions (must match crazyflies.yaml)
        self.initial_positions = {
            'cf1': [0.0, 0.0, 0.0],
            'cf2': [2.5, 0.0, 0.0],
        }

        # Wait for all services
        self.get_logger().info('Waiting for Crazyflie services...')
        for drone in self.drones:
            for cli in [self.takeoff_clients[drone],
                        self.land_clients[drone],
                        self.notify_clients[drone]]:
                while not cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(
                        f'Waiting for {drone} {cli.srv_name}...')
        self.get_logger().info('All services available!')
```

`initial_positions` must match `crazyflies.yaml` — trajectory math references these to compute start and end positions. The nested service-waiting loop checks all three services for each drone.

### 5.5: Service methods — per-drone wrappers

The service call logic follows the same request/response pattern introduced in Example 1, but each method takes a `drone` argument to select the correct client from the per-drone dictionaries. Add these methods inside the `MultiDroneFlight` class:

```python
    def takeoff(self, drone: str, height: float, duration: float):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = height
        req.duration = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        future = self.takeoff_clients[drone].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info(f'{drone}: Takeoff to {height}m')

    def land(self, drone: str, duration: float):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        future = self.land_clients[drone].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info(f'{drone}: Landing')

    def notify_setpoints_stop(self, drone: str):
        req = NotifySetpointsStop.Request()
        req.group_mask = 0
        req.remain_valid_millisecs = 100
        future = self.notify_clients[drone].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
```

### 5.6: publish_full_state — per-drone

Same expanded signature from Example 2, now with a `drone` argument.

```python
    def publish_full_state(self, drone: str, x: float, y: float,
                           z: float, vx: float = 0.0, vy: float = 0.0,
                           vz: float = 0.0, ax: float = 0.0,
                           ay: float = 0.0, az: float = 0.0,
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

        self.fullstate_pubs[drone].publish(msg)
```

The only change from Example 2: `self.fullstate_pubs[drone].publish(msg)` selects the right publisher from the dictionary.

### 5.7: Trajectory computation

**Square trajectory for cf1** — same segment decomposition logic from Example 2, parameterized with cf1's parameters and start position:

```python
    def square_trajectory(self, elapsed: float):
        if elapsed < 0 or elapsed >= self.square_duration:
            return None

        seg_dur = self.square_segment_duration
        seg_idx = int(elapsed / seg_dur)
        seg_t = elapsed - seg_idx * seg_dur
        frac = seg_t / seg_dur

        x0, y0 = self.initial_positions['cf1'][0], self.initial_positions['cf1'][1]

        if seg_idx == 0:      # +X
            x, y = x0 + self.square_side * frac, y0
            vx, vy = self.speed, 0.0
        elif seg_idx == 1:    # +Y
            x, y = x0 + self.square_side, y0 + self.square_side * frac
            vx, vy = 0.0, self.speed
        elif seg_idx == 2:    # -X
            x, y = x0 + self.square_side * (1.0 - frac), y0 + self.square_side
            vx, vy = -self.speed, 0.0
        else:                 # -Y
            x, y = x0, y0 + self.square_side * (1.0 - frac)
            vx, vy = 0.0, -self.speed

        return Setpoint(x=x, y=y, z=self.flight_height, vx=vx, vy=vy)
```

**Circle trajectory for cf2** — new concept. A circle of radius `r` centered at `(cx, cy)` parameterized by angle `θ`:

```python
    def circle_trajectory(self, elapsed: float):
        if elapsed < 0 or elapsed > self.circle_duration:
            return None

        radius = self.circle_radius

        # Center: cf2's initial position (2.5, 0) is the circle's
        # leftmost point. Transition moves cf2 right to the rightmost
        # point before the circle starts. Rightmost = cx + radius.
        # Since leftmost (2.5) = cx - radius, cx = 2.5 + 1.0 = 3.5.
        cx = self.initial_positions['cf2'][0] + radius
        cy = self.initial_positions['cf2'][1]

        # Angle increases linearly from 0 to 2π over the full duration
        angle = (elapsed / self.circle_duration) * 2.0 * math.pi

        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)

        # Tangent velocity: derivative of position w.r.t. time
        # d/dt[cos(θ)] = -sin(θ) * dθ/dt,  and dθ/dt = speed / radius
        vx = -self.speed * math.sin(angle)
        vy = self.speed * math.cos(angle)

        return Setpoint(x=x, y=y, z=self.flight_height,
                        vx=vx, vy=vy, yaw=angle)
```

**How the circle math works:**

**Center placement:** cf2 transitions from its initial position (2.5, 0) to the circle's **rightmost** point (4.5, 0) before the circle starts. The rightmost point of a circle centered at `(cx, cy)` is `(cx + r, cy)`. So `4.5 = cx + 1.0` → `cx = 3.5`. The leftmost point is `cx - r = 2.5` — which is where cf2 started. The transition phase bridges the gap before circling begins. The circle center is at (3.5, 0).

**Angle progression:** `angle = (elapsed / duration) * 2π`. At `elapsed=0`, angle=0 (rightmost point, x=4.5). At `elapsed=duration/4`, angle=π/2 (top, y=1.0). At `elapsed=duration/2`, angle=π (leftmost, x=2.5). At `elapsed=duration`, angle=2π (back to rightmost — one full lap). The drone flies counter-clockwise: angle increases, cos decreases, sin increases.

**Position:** `(cx + r·cos(θ), cy + r·sin(θ))`. At angle=0: (3.5 + 1.0, 0) = (4.5, 0) — the rightmost point, matching where the transition phase placed the drone. At angle=π: (3.5 - 1.0, 0) = (2.5, 0) — the leftmost point, which is cf2's initial position. The circle spans x ∈ [2.5, 4.5], safely to the right of cf1's square (x ∈ [0, 1.5]).

**Tangent velocity:** The derivative of `cos(θ(t))` is `-sin(θ) · dθ/dt`. Since `dθ/dt = 2π/duration = speed/radius`, the radius cancels: `vx = -r · sin(θ) · speed/r = -speed · sin(θ)`. Similarly, `vy = speed · cos(θ)`. At angle=0: `vx=0, vy=+speed` — moving straight up. At angle=π/2: `vx=-speed, vy=0` — moving left. This is counter-clockwise motion.

**Yaw tracking:** `yaw=angle` means the drone's heading rotates with the circle. At angle=0 (rightmost point, x=4.5), yaw=0 means the drone faces +X while its velocity is purely +Y (upward tangent) — it flies **sideways** relative to its nose. This is perfectly normal for a quadrotor. To have the drone face its direction of motion, use `yaw = angle + math.pi/2` — this rotates the heading by 90° to align with the tangent. Try both and observe the difference in RViz.

### 5.8: Phase helper — run a streaming phase for all drones

This helper reduces code duplication in `main()`. Instead of writing a `while` loop for every phase, "setpoint functions" are defined and pass them to a reusable loop.

```python
    def run_streaming_phase(self, phase_name: str, duration: float,
                            get_setpoint_fn):
        """Run a streaming phase for all drones simultaneously.

        Args:
            phase_name: Display name for logging.
            duration: How long the phase lasts (seconds).
            get_setpoint_fn: Function(drone, elapsed) -> Setpoint or None.
        """
        self.get_logger().info(f'=== {phase_name} ({duration:.1f}s) ===')
        rate = RateController(self.dt)
        rate.start()
        start_time = time.monotonic()

        while time.monotonic() - start_time < duration:
            elapsed = time.monotonic() - start_time
            for drone in self.drones:
                sp = get_setpoint_fn(drone, elapsed)
                if sp is not None:
                    self.publish_full_state(
                        drone, sp.x, sp.y, sp.z,
                        sp.vx, sp.vy, sp.vz,
                        sp.ax, sp.ay, sp.az,
                        sp.yaw)
            rate.sleep()

        if rate.overruns:
            self.get_logger().warning(
                f'{phase_name}: {rate.overruns} overruns at 30Hz')
```

`get_setpoint_fn(drone, elapsed) -> Setpoint or None` — a function that takes a drone name and elapsed time and returns the setpoint. Each phase (stabilize, transition, fly) defines its own function. The inner loop iterates over all drones, computing and publishing a setpoint for each on every tick. Both drones receive setpoints at 30Hz.

### 5.9: Setpoint functions for each phase

**Stabilize phase** — hover at takeoff positions:

```python
    def hover_at_takeoff(self, drone, elapsed):
        pos = self.initial_positions[drone]
        return Setpoint(x=pos[0], y=pos[1], z=self.flight_height)
```

**Transition to start positions** — cf1 stays at origin; cf2 moves right to the circle's starting point (the rightmost point of the circle at x=4.5, center at x=3.5):

```python
    def transition_to_start(self, drone, elapsed):
        pos = self.initial_positions[drone]
        if drone == 'cf1':
            target_x, target_y = pos[0], pos[1]
        else:  # cf2 moves to circle's rightmost point
            cx = pos[0] + self.circle_radius      # center x = 2.5 + 1.0 = 3.5
            target_x = cx + self.circle_radius     # rightmost = 3.5 + 1.0 = 4.5
            target_y = pos[1]

        frac = min(elapsed / self.transition_duration, 1.0)
        start_x, start_y = pos[0], pos[1]
        x = start_x + (target_x - start_x) * frac
        y = start_y + (target_y - start_y) * frac
        return Setpoint(x=x, y=y, z=self.flight_height)
```

`frac = min(elapsed / transition_duration, 1.0)` — linear interpolation, clamped at 1.0. cf1: start and target are the same (origin), so the drone hovers in place but still receives setpoints — this keeps the phase structure uniform. cf2: moves from x=2.5 to x=4.5 over `transition_duration` seconds.

**Fly phase** — routes each drone to its trajectory. When a trajectory finishes (returns `None`), the drone **hovers at its end position** while the other continues. This is the hover-wait pattern:

```python
    def fly_trajectory(self, drone, elapsed):
        if drone == 'cf1':
            sp = self.square_trajectory(elapsed)
            if sp is not None:
                return sp
            # Square finished — hover at start corner (origin)
            pos = self.initial_positions['cf1']
            return Setpoint(x=pos[0], y=pos[1], z=self.flight_height)
        else:  # cf2
            sp = self.circle_trajectory(elapsed)
            if sp is not None:
                return sp
            # Circle finished — hover at rightmost point
            cx = self.initial_positions['cf2'][0] + self.circle_radius
            end_x = cx + self.circle_radius
            end_y = self.initial_positions['cf2'][1]
            return Setpoint(x=end_x, y=end_y, z=self.flight_height)
```

**Why not just stop publishing when a trajectory finishes?** Stopping setpoints triggers the firmware's setpoint timeout — the drone would enter emergency stop mode. On real hardware, this is a crash. Publishing a hover setpoint keeps the drone safely in LLC mode while waiting. The fly phase runs for `max(square_duration, circle_duration)` — the longer of the two trajectories. cf1 finishes first (~20.0s) and hovers at the origin for the remaining ~0.9s while cf2 finishes its circle (~20.9s).

**Transition to landing positions** — both drones return to their initial XY positions:

```python
    def transition_to_land(self, drone, elapsed):
        pos = self.initial_positions[drone]
        if drone == 'cf1':
            start_x, start_y = pos[0], pos[1]    # square ends at origin
        else:
            cx = pos[0] + self.circle_radius
            start_x = cx + self.circle_radius     # circle ends at rightmost
            start_y = pos[1]

        target_x, target_y = pos[0], pos[1]
        frac = min(elapsed / self.transition_duration, 1.0)
        x = start_x + (target_x - start_x) * frac
        y = start_y + (target_y - start_y) * frac
        return Setpoint(x=x, y=y, z=self.flight_height)
```

### 5.10: main() function

```python
def main():
    rclpy.init()
    node = MultiDroneFlight()

    with FlightSession(node, node.drones,
                       land_duration=node.land_duration,
                       post_land_wait=2.0) as flight:

        # === Phase 1: Takeoff both drones ===
        print("\n" + "="*50)
        print("PHASE 1: TAKEOFF")
        print("="*50)
        for drone in node.drones:
            flight.takeoff(drone, node.flight_height,
                           node.takeoff_duration)
        time.sleep(node.takeoff_duration + 1.5)

        # === Phase 2: Stabilize ===
        print("\n" + "="*50)
        print("PHASE 2: STABILIZE")
        print("="*50)
        node.run_streaming_phase(
            "Stabilize", node.stabilize_duration,
            node.hover_at_takeoff)

        # === Phase 3: Move to trajectory start positions ===
        print("\n" + "="*50)
        print("PHASE 3: MOVE TO START")
        print("="*50)
        node.run_streaming_phase(
            "Move to start", node.transition_duration,
            node.transition_to_start)
        time.sleep(node.phase_pause)
```

Each drone gets its own `takeoff()` call. `FlightSession.takeoff()` adds each to the airborne set. The stabilize phase has both drones hover at their takeoff positions for 2s before transitioning.

```python
        # === Phase 4: Fly trajectories simultaneously ===
        print("\n" + "="*50)
        print("PHASE 4: FLY")
        print("="*50)
        fly_duration = max(node.square_duration, node.circle_duration)
        print(f"  cf1: square, {node.square_duration:.1f}s")
        print(f"  cf2: circle, {node.circle_duration:.1f}s")
        print(f"  Running for {fly_duration:.1f}s (longest trajectory)")

        node.run_streaming_phase("Fly", fly_duration,
                                 node.fly_trajectory)
        time.sleep(node.phase_pause)
```

`fly_duration = max(square_duration, circle_duration)` runs for the longer trajectory. `fly_trajectory()` handles the hover-wait: the drone that finishes first hovers at its end position.

```python
        # === Phase 5: Return to initial positions ===
        print("\n" + "="*50)
        print("PHASE 5: RETURN TO START")
        print("="*50)
        node.run_streaming_phase(
            "Return to start", node.transition_duration,
            node.transition_to_land)
        time.sleep(node.phase_pause)

        # === Phase 6: Land all drones ===
        print("\n" + "="*50)
        print("PHASE 6: LAND")
        print("="*50)
        for drone in node.drones:
            node.notify_setpoints_stop(drone)
        time.sleep(0.1)
        for drone in node.drones:
            node.land(drone, node.land_duration)

        time.sleep(node.land_duration + 1.0)
        flight._airborne.clear()

        print("\n" + "="*50)
        print("FLIGHT COMPLETE!")
        print("="*50)

    node.destroy_node()
    rclpy.shutdown()
```

Phase 6 calls `notify_setpoints_stop` and `land` for each drone individually. `flight._airborne.clear()` empties the airborne set so `__exit__` does not re-land.

### 5.11: Entry point guard

```python
if __name__ == '__main__':
    main()
```

---

## Section 6: Register the Entry Point

Edit `setup.py`:

```python
from setuptools import setup
from glob import glob

package_name = 'sim_multidrone'

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
    description='Example 3: Sim multidrone control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'multi_flight = sim_multidrone.multi_flight:main',
        ],
    },
)
```

`glob('config/*')` picks up all files in `config/`: `crazyflies.yaml`, `sim_multidrone.rviz`, `cf1.urdf`, and `cf2.urdf`. All four must be installed as data files so the launch file and RViz can find them at runtime.

---

## Section 7: Build and Test

### 7.1: Build

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
colcon build --symlink-install
```

### 7.2: Launch and run

**Terminal 1** — sim server + RViz with the custom config:

```bash
source install/setup.bash
ros2 launch sim_multidrone sim_multidrone.launch.py rviz:=true
```

**Terminal 2** — flight script:

```bash
cd $CRAZYFLIE_TUTORIAL/tutorial_ws
source install/setup.bash
ros2 run sim_multidrone multi_flight --ros-args -p use_sim_time:=True
```

**Expected behavior in RViz:**
1. Two drones appear: cf1 at (0, 0, 0), cf2 at (2.5, 0, 0)
2. Both take off to z=0.5 simultaneously
3. Stabilize: both hover for 2s
4. Move to start: cf1 stays at origin; cf2 moves right to x=4.5
5. Fly: cf1 traces a 1.5m square around the origin; cf2 traces a 1.0m-radius circle centered at x=3.5. cf1 finishes slightly before cf2 and hovers at the origin while cf2 completes its last ~0.9s
6. Return: both move back to their initial XY positions
7. Land: both descend to z=0

---

## Section 8: Architecture Recap

### Multi-drone ROS2 communication

```
Terminal 2: multi_flight.py
    │
    ├── /cf1/cmd_full_state (30Hz) ──► crazyflie_server ──► cf1 (sim)
    ├── /cf2/cmd_full_state (30Hz) ──► crazyflie_server ──► cf2 (sim)
    ├── /cf1/takeoff, /cf1/land ──────► crazyflie_server
    └── /cf2/takeoff, /cf2/land ──────► crazyflie_server
                                                │
                                          TF: world → cf1
                                          TF: world → cf2
                                          /cf1/robot_description, /cf2/robot_description
                                                │
Terminal 1: RViz ◄────────────────────────────┘
```

### Key concept: hover-wait synchronization

When drones fly different trajectories with different durations, the one that finishes first must wait for the others before proceeding to the next phase. The `fly_trajectory()` method implements this: when a drone's trajectory returns `None` (finished), it falls through to a hover setpoint at the end position. The drone stays in LLC mode, safely hovering, while the `run_streaming_phase` loop continues ticking at 30Hz for the remaining drones. This pattern is essential for hardware — stopping setpoints triggers emergency shutdown after ~100ms.

### Key concept: the dictionary pattern for multi-drone

Using dictionaries (`self.takeoff_clients['cf1']`, `self.fullstate_pubs['cf2']`) instead of individual variables (`self.cf1_takeoff`, `self.cf2_takeoff`) makes the code scale. Adding a third drone is a one-line change to `self.drones` plus a third entry in `crazyflies.yaml` and `initial_positions`. The `for drone in self.drones:` loops handle everything else automatically.

---

## Verification Checklist

- [ ] `colcon build --symlink-install` succeeds
- [ ] `ros2 launch sim_multidrone sim_multidrone.launch.py rviz:=true` starts the sim server and RViz with the custom config
- [ ] Both drones appear in RViz with correct robot models (if not, check that the sim server is publishing `/cf1/robot_description` and `/cf2/robot_description`)
- [ ] `ros2 run sim_multidrone multi_flight --ros-args -p use_sim_time:=True` completes without errors
- [ ] cf1 traces a square; cf2 traces a circle
- [ ] cf1 hovers at the origin while cf2 finishes (hover-wait)
- [ ] Both drones return to their initial positions before landing
- [ ] Ctrl+C during the fly phase triggers emergency landing for both

---

## Potential Issues

- **One drone does not appear in RViz** — check that the sim server is running and publishing robot descriptions: `ros2 topic list | grep robot_description` should show both `/cf1/robot_description` and `/cf2/robot_description`. Also verify TF is publishing for both drones: `ros2 topic echo /tf | grep cf`.
- **Drones collide** — the `initial_position` values provide 2.5m separation. If trajectories overlap, increase `cf2.initial_position[0]` or reduce trajectory sizes.
- **Circle looks oval** — check the tangent velocity formula: `vx = -speed * sin(angle)`, `vy = speed * cos(angle)`. Swapping sin/cos or sign errors produce distorted paths.
- **One drone freezes after its trajectory** — the `fly_trajectory()` fallback may not be returning a hover setpoint. Check the `if sp is not None` logic.
- **Flight script hangs on "Waiting for services"** — check that both drones are `enabled: true` in `crazyflies.yaml` and the sim server started fully.
