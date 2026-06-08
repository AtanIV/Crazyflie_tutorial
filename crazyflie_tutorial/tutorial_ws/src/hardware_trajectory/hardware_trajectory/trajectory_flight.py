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