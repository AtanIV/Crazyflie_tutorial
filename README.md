# Crazyflie + Crazyswarm2 + Vicon Mocap — Step-by-Step Tutorial

A self-contained tutorial series for learning to control Crazyflie 2.1 drones using Crazyswarm2 with Vicon motion capture via ROS2 Humble. All scripts, configs, and launch files are provided in full with explanations. This tutorial will mainly focus on the usage of the firmware and toolchains and therefore only involve relatively simple trajectories and control methods. After getting familiar with the ros2 crazyswarm toolchain, the reader will be able to implement and deploy more complicated controllers and observers using the same framework.

> **Please do use AI while going through this tutorial!** Some parts of this tutorial, like the viewer in example 4, include helper functions and overcomplicated settings just for style and completeness and are unrelated to core usage of ROS2 and Crazyflie. Going through the visualization and state machine function pieces can be **Tedious** and unnecessary. Therefore usage of AI to just get the hang of it and focusing on the important parts is strongly advised!

---

## Prerequisites

- Ubuntu 22.04 LTS (bare metal or dual-boot; VM strongly discouraged)
- ROS2 Humble (desktop-full)
- Crazyradio 2.0 (PA mode firmware)
- Crazyflie 2.1 drone(s) with reflective markers
- Vicon motion capture system (for hardware examples)

The tutorial begins with environment setup and proceeds through six progressively more complex examples, from a single simulated drone hover to coordinated multi-drone trajectory flight on real hardware.

---

## Tutorial Structure

### Part 0 — Prerequisites and Setup

Covers installing Ubuntu 22.04 and ROS2 Humble, cloning and building the required repositories (Crazyswarm2, cflib, cfclient, crazyflie-firmware), setting up the Crazyradio with PA mode firmware, flashing drone firmware and assigning unique radio addresses, configuring the Vicon Tracker software, and verifying the mocap data pipeline.

**Tutorial file:** [`tutorial_part0_prerequisites.md`](tutorial_part0_prerequisites.md)

### Example 1 — Sim Takeoff, Hover, and Landing

A single simulated Crazyflie takes off, hovers at 0.5m for 5 seconds, and lands. Introduces the ROS2 node architecture, the Crazyswarm2 simulation backend, RViz visualization, and the fundamental patterns reused throughout all later examples: `RateController` (phase-locked 30Hz timer for the examples), `FlightSession` (context manager guaranteeing landing on any exit), and `publish_full_state` (streaming low-level setpoints). Also covers the critical distinction between High-Level Commander (HLC) services and Low-Level Commander (LLC) streaming, and the `notify_setpoints_stop` transition between them.

**Package:** `sim_takeoff` &nbsp;|&nbsp; **Tutorial file:** [`tutorial_example1_sim_takeoff.md`](tutorial_example1_sim_takeoff.md)

### Example 2 — Sim Trajectory Flight

Extends the hover script to fly a square trajectory with four straight segments. Introduces the `Setpoint` dataclass for clean setpoint definitions, trajectory decomposition into timed segments with linear interpolation, and velocity feedforward — why adding tangent velocity to position commands dramatically improves tracking (sharp corners instead of rounded blobs). Also explains the Mellinger controller (used in simulation) and notes that the `cmd_full_state` interface is identical with PID (used on hardware).

**Package:** `sim_trajectory` &nbsp;|&nbsp; **Tutorial file:** [`tutorial_example2_sim_trajectory.md`](tutorial_example2_sim_trajectory.md)

### Example 3 — Sim Multidrone Control

Controls two simulated drones simultaneously: cf1 flies a 1.5m square, cf2 flies a 1.0m-radius circle. Introduces multi-drone configuration in `crazyflies.yaml`, per-drone publishers and service clients stored in dictionaries, the lambda capture pattern for per-drone ROS2 callbacks, circle trajectory math with tangent velocity and yaw tracking (with only one marker per drone, yaw is unobservable therefore open loop), the hover-wait synchronization pattern (a drone that finishes first hovers while others complete their trajectories), and custom RViz configuration with multiple robot models and URDF files.

**Package:** `sim_multidrone` &nbsp;|&nbsp; **Tutorial file:** [`tutorial_example3_sim_multidrone.md`](tutorial_example3_sim_multidrone.md)

### Example 4 — Vicon Streaming and Visualization

Shifts from controlling drones to building a real-time 3D visualization tool. Writes a vispy-based viewer that subscribes to the `/poses` topic and renders drone positions with colored trails, floor shadows, drop-lines, text labels, a HUD overlay (frame counter, FPS, drone count), a recording button for CSV export, and optional obstacle overlay from CSV files. Also writes a `fake_mocap` publisher that generates simulated marker data for testing without Vicon hardware. Introduces thread-safe data bridging between ROS2 callbacks and the GUI event loop, BEST_EFFORT QoS for high-rate sensor data, and the `NamedPoseArray` message format. This viewer is reused in all subsequent hardware examples.

**Package:** `vicon_viewer` &nbsp;|&nbsp; **Tutorial file:** [`tutorial_example4_vicon_viewer.md`](tutorial_example4_vicon_viewer.md)

### Example 5 — Hardware Connection and Hover Test

The first physical flight. Connects to a real Crazyflie via Crazyradio, performs comprehensive pre-flight verification (mocap pairing, EKF convergence monitoring, supervisor self-test, battery check), and executes a keyboard-gated hover at 30cm for 5 seconds. The drone must be physically placed facing world +X before takeoff — single-marker Vicon provides position only (no orientation), so yaw is unobservable and cannot be corrected automatically. Introduces the keyboard-gated safety state machine, five emergency trigger types (pose timeout, position bounds, battery critical, drone tumbled, keyboard abort), the emergency landing fallback chain (hold position → notify_stop → land → motor cutoff), the `termios`/`tty` keyboard thread for raw terminal input, and CSV flight logging with source tracking (ekf/vicon/cmd). A manual pre-flight verification step using `ros2 topic echo` is required before every flight.

**Package:** `hardware_hover` &nbsp;|&nbsp; **Tutorial file:** [`tutorial_example5_hardware_hover.md`](tutorial_example5_hardware_hover.md)

### Example 6 — Hardware Trajectory Flight

The capstone example, split into two parts:

**6A (single-drone):** Extends the Example 5 hover script with flight states: move-to-start (linear interpolation from hover position to trajectory corner with velocity feedforward), hover-at-start (waiting for the 'G' key), and fly (rectangle trajectory with live progress display showing segment number, elapsed time, and real-time tracking error). The trajectory math is the same as Example 2, adapted for hardware with configurable parameters. Same placement requirement as Example 5 — drone must face world +X before takeoff.

**6B (multi-drone):** Two drones fly simultaneously — cf1 on a rectangle, cf3 on a circle. Both drones must be physically placed facing world +X before takeoff (single-marker Vicon provides no per-drone orientation). The hover-wait pattern ensures the drone that finishes first hovers safely while the other completes. Per-drone safety checks run on every iteration — a violation on any drone triggers emergency landing for the entire swarm. Circle trajectory math is adapted from Example 3 for hardware execution.

**Package:** `hardware_trajectory` &nbsp;|&nbsp; **Tutorial file:** [`tutorial_example6_hardware_trajectory.md`](tutorial_example6_hardware_trajectory.md)

---

## Expected File Tree

After completing all tutorials, the workspace will have the following structure:

```
$CRAZYFLIE_TUTORIAL/
│
├── crazyswarm2_repo/                  # Cloned CS2, colcon-built
│   └── crazyswarm2/
│       ├── build/
│       ├── install/
│       └── src/                       # CS2 packages
│
├── crazyflie-lib-python/              # cflib (pip install -e .)
├── crazyflie-clients-python/          # cfclient (pip install -e .)
├── crazyflie-firmware/                # Firmware + cffirmware Python bindings
│   └── build/
│       └── cffirmware.py
│
├── mocap_test/                        # Part 0 mocap verification
│   ├── crazyflies.yaml
│   └── motion_capture.yaml
│
├── obstacles_test.csv                 # Example 4 obstacle test data
│
└── tutorial_ws/                       # Shared ROS2 workspace
    ├── build/
    ├── install/
    ├── log/
    └── src/
        │
        ├── sim_takeoff/               # Example 1
        │   ├── package.xml
        │   ├── setup.py / setup.cfg
        │   ├── config/crazyflies.yaml
        │   ├── launch/sim_takeoff.launch.py
        │   └── sim_takeoff/
        │       ├── __init__.py
        │       └── takeoff_hover_land.py
        │
        ├── sim_trajectory/            # Example 2
        │   ├── package.xml
        │   ├── setup.py / setup.cfg
        │   ├── config/crazyflies.yaml
        │   ├── launch/sim_trajectory.launch.py
        │   └── sim_trajectory/
        │       ├── __init__.py
        │       └── trajectory_flight.py
        │
        ├── sim_multidrone/            # Example 3
        │   ├── package.xml
        │   ├── setup.py / setup.cfg
        │   ├── config/
        │   │   ├── crazyflies.yaml
        │   │   ├── sim_multidrone.rviz
        │   │   ├── cf1.urdf
        │   │   └── cf2.urdf
        │   ├── launch/sim_multidrone.launch.py
        │   └── sim_multidrone/
        │       ├── __init__.py
        │       └── multi_flight.py
        │
        ├── vicon_viewer/              # Example 4
        │   ├── package.xml
        │   ├── setup.py / setup.cfg
        │   ├── config/crazyflies.yaml
        │   ├── launch/viewer.launch.py
        │   └── vicon_viewer/
        │       ├── __init__.py
        │       ├── drone_viewer.py
        │       └── fake_mocap.py
        │
        ├── hardware_hover/            # Example 5
        │   ├── package.xml
        │   ├── setup.py / setup.cfg
        │   ├── config/
        │   │   ├── crazyflies.yaml
        │   │   ├── motion_capture.yaml
        │   │   └── flight_config.yaml
        │   ├── launch/hardware_hover.launch.py
        │   ├── logs/                  # (generated at runtime)
        │   └── hardware_hover/
        │       ├── __init__.py
        │       └── hover_test.py
        │
        └── hardware_trajectory/       # Example 6
            ├── package.xml
            ├── setup.py / setup.cfg
            ├── config/
            │   ├── crazyflies.yaml
            │   ├── crazyflies_multi.yaml
            │   ├── motion_capture.yaml
            │   ├── flight_config.yaml
            │   └── flight_config_multi.yaml
            ├── launch/
            │   ├── trajectory_flight.launch.py
            │   └── trajectory_flight_multi.launch.py
            ├── logs/                  # (generated at runtime)
            └── hardware_trajectory/
                ├── __init__.py
                ├── trajectory_flight.py
                └── trajectory_flight2.py
```

---

## How to Use This Tutorial

1. Start with **Part 0** (Prerequisites and Setup). Work through each section in order — later sections depend on earlier ones.

2. Proceed through the examples in sequence (1 through 6). Each example builds on concepts from the previous ones. However, each example is **self-contained** — all code is written out in full, so the reader does not need to reference earlier examples for code.

3. For each example:
   - Read the overview to understand the goal
   - Follow the step-by-step instructions to create files and write code
   - Build with `colcon build --symlink-install`
   - Run the tests as documented
   - Review the "Key Concepts" or "How It Works" section to solidify understanding
   - Check the "Build and Test" verification steps and "Potential Issues" before moving on

4. **Hardware examples (5-6) require extra caution.** Always perform the manual pre-flight verification steps before running any flight script. The keyboard-gated state machine requires explicit keypress approval at each phase — the script will never proceed autonomously from one flight phase to the next. Press 'E' at any time for emergency landing.

---

## Key Design Decisions

- **Sim visualization:** RViz (NiceGUI disabled due to compatibility issues)
- **Hardware visualization:** Custom vispy viewer (built in Example 4)
- **Hardware backend:** `cflib` (Python `crazyflie_server.py`; C++ backend has bugs with Crazyradio 2.0 in PA mode)
- **Sim controller:** Mellinger (`controller: 2` in `crazyflies.yaml`)
- **Hardware controller:** PID (`controller: 1` in `crazyflies.yaml`) — `cmd_full_state` interface is identical regardless of controller choice
- **Control method:** `cmd_full_state` streaming at 30Hz across all examples
- **Mocap pipeline:** Native Crazyswarm2 `motion_capture_tracking_node` + `librigidbodytracker` (no custom Vicon bridge)
- **Kalman filter reset:** Disabled (single-marker Vicon provides no yaw reference; resetting zeros the yaw estimate with no way to recover)
- **Yaw handling:** No software yaw calibration. Single-marker Vicon makes yaw unobservable (position-only measurement). The drone is physically placed facing world +X before takeoff; the position loop tolerates moderate placement error. A multi-marker rigid body would make yaw observable if automatic correction is needed.
- **Launch files:** Standalone `Node()` definitions; flight scripts never bundled; viewer optionally bundled via `viewer:=true`
- **Recommended workflow:** Separate terminals for each node so output is visible for debugging; launch files provided as convenience shortcuts
