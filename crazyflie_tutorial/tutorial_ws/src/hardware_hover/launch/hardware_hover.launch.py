import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hardware_hover')
    cs2_share = get_package_share_directory('crazyflie')

    # Paths to config files
    crazyflies_path = os.path.join(pkg_dir, 'config', 'crazyflies.yaml')
    mocap_yaml_path = os.path.join(pkg_dir, 'config', 'motion_capture.yaml')
    server_yaml_path = os.path.join(cs2_share, 'config', 'server.yaml')
    urdf_path = os.path.join(cs2_share, 'urdf', 'crazyflie_description.urdf')

    # Load configs
    with open(crazyflies_path) as f:
        crazyflies = yaml.safe_load(f)
    with open(mocap_yaml_path) as f:
        mocap_cfg = yaml.safe_load(f)
    with open(server_yaml_path) as f:
        server_cfg = yaml.safe_load(f)
    with open(urdf_path) as f:
        robot_desc = f.read()

    # Build mocap parameters (marker configs + Vicon connection)
    mocap_params = mocap_cfg['/motion_capture_tracking']['ros__parameters']
    mocap_params['rigid_bodies'] = {}
    for key, value in crazyflies['robots'].items():
        if value['enabled']:
            robot_type = crazyflies['robot_types'][value['type']]
            mc = robot_type['motion_capture']
            if mc['enabled'] and mc.get('tracking') == 'librigidbodytracker':
                mocap_params['rigid_bodies'][key] = {
                    'initial_position': value['initial_position'],
                    'marker': mc['marker'],
                    'dynamics': mc['dynamics'],
                }

    # Build server parameters
    server_params = [crazyflies]
    server_params.append(
        server_cfg['/crazyflie_server']['ros__parameters'])
    server_params[1]['robot_description'] = robot_desc
    server_params[1]['poses_qos_deadline'] = \
        mocap_params['topics']['poses']['qos']['deadline']

    # Nodes
    mocap_node = Node(
        package='motion_capture_tracking',
        executable='motion_capture_tracking_node',
        name='motion_capture_tracking',
        output='screen',
        parameters=[mocap_params],
    )

    server_node = Node(
        package='crazyflie',
        executable='crazyflie_server.py',
        name='crazyflie_server',
        output='screen',
        parameters=server_params,
    )

    viewer_node = Node(
        package='vicon_viewer',
        executable='drone_viewer',
        name='drone_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='false'),
        mocap_node,
        server_node,
        viewer_node,
    ])