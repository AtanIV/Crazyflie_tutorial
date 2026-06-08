"""
Simulated takeoff, hover, and landing for one Crazyflie.

Uses the Crazyswarm2 simulation backend. The drone takes off to 0.5m,
hovers for 5 seconds using streaming cmd_full_state setpoints, then lands.
"""

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import FullState
from builtin_interfaces.msg import Duration
import time
import math

class RateController:
    """Phase-locked rate limiter with skip-on-overrun.

    Anchors sleep to absolute monotonic time so the loop period stays at
    self.period on average, regardless of work duration. If a single
    iteration exceeds the period, the next slot is skipped — this avoids
    bursting the radio after a hiccup.
    """
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
    """Context manager that lands the drone on any exit path.

    Tracks whether takeoff was issued so land is not attempted on a drone
    that never left the ground. Cleanup runs for normal exit,
    KeyboardInterrupt, and any unhandled exception.
    """

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

        return False  # don't suppress exceptions

class TakeoffHoverLand(Node):

    def __init__(self):
        super().__init__('takeoff_hover_land')

                # Flight parameters
        self.drone = 'cf1'
        self.flight_height = 0.5          # meters
        self.hover_duration = 5.0         # seconds
        self.dt = 1.0 / 30.0              # 30 Hz control rate

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
                           yaw: float = 0.0):
        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        msg.acc.x = 0.0
        msg.acc.y = 0.0
        msg.acc.z = 0.0

        self.fullstate_pub.publish(msg)
    
def main():
    rclpy.init()
    node = TakeoffHoverLand()

    with FlightSession(node, 'cf1',
                       land_duration=2.5) as flight:

        # === Phase 1: Takeoff ===
        print("Taking off...")
        flight.takeoff(node.flight_height, 2.5)
        time.sleep(3.0)  # wait for takeoff to complete + settle

        # === Phase 2: Hover (streaming setpoints) ===
        print(f"Hovering at {node.flight_height}m for "
              f"{node.hover_duration}s...")
        rate = RateController(node.dt)
        rate.start()
        hover_start = time.monotonic()
        while time.monotonic() - hover_start < node.hover_duration:
            node.publish_full_state(
                0.0, 0.0, node.flight_height, yaw=0.0)
            rate.sleep()

        if rate.overruns:
            node.get_logger().warning(
                f'{rate.overruns} loop overruns at 30Hz')
        
        # === Phase 3: Land ===
        print("Landing...")
        node.notify_setpoints_stop()
        time.sleep(0.1)
        node.land(2.5)
        time.sleep(3.0)
        flight._airborne = False  # prevent __exit__ from re-landing

        print("Flight complete!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()