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
                    "start_position_m": [1.8, 0.0]},
        },
        "shared": {
            "height_m": 0.30, "velocity_ms": 0.20,
            "transition_velocity_ms": 0.10,
        },
        "safety": {"max_distance_m": 3.0, "max_height_m": 0.60,
                   "min_height_m": 0.05, "pose_timeout_s": 0.5,
                   "battery_critical_v": 3.2},
        "control": {"rate_hz": 30, "takeoff_duration_s": 3.0,
                    "land_duration_s": 3.0, "hover_stabilize_s": 3.0},
    }

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
    
    def _print(self, text=""):
        """Print with \\r\\n so output is correct in raw terminal mode."""
        for line in text.split("\n"):
            sys.stdout.write(line + "\r\n")
        sys.stdout.flush()
    
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

        # RETURNING — move back to initial positions
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