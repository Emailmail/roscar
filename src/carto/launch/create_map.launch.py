"""Cartographer 2D SLAM mapping — sensors + Cartographer in mapping mode."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dm_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dm_imu'), 'launch', 'dm_imu.launch.py')
        ),
    )

    ld06_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld06.launch.py')
        ),
    )

    carto_config_dir = os.path.join(get_package_share_directory('carto'), 'config')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', carto_config_dir,
            '-configuration_basename', 'mapping.lua',
        ],
        remappings=[
            ('imu', 'imu/data'),
            ('echoes', 'scan'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}],
    )

    # TF: base_link → base_laser (Z=0.18, LiDAR mounting height).
    # Published here explicitly so the chain never depends on the ld06 submodule alone.
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='carto_base_link_to_base_laser',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.18',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'base_laser',
        ],
    )

    base_link_to_imu_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='carto_base_link_to_imu_link',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
        ],
    )

    return LaunchDescription([
        dm_imu_launch,
        ld06_launch,
        cartographer_node,
        occupancy_grid_node,
        base_link_to_laser,
        base_link_to_imu_link,
    ])
