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