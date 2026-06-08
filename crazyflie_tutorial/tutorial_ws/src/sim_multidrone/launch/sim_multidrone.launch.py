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