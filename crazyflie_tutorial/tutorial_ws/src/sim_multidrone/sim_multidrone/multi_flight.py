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
    
    def hover_at_takeoff(self, drone, elapsed):
        pos = self.initial_positions[drone]
        return Setpoint(x=pos[0], y=pos[1], z=self.flight_height)
    
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
        time.sleep(node.phase_pause)\
        
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

if __name__ == '__main__':
    main()