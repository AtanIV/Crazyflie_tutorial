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

if __name__ == '__main__':
    main()
