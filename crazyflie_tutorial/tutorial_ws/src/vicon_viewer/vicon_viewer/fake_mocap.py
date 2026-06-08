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