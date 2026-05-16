"""Cartographer 2D SLAM mapping — sensors + Cartographer, saves map to src/map/."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

MAP_DIR = '/home/relog/roscar/src/map'


def generate_launch_description():
    map_save_path_arg = DeclareLaunchArgument(
        'map_save_path',
        default_value=MAP_DIR,
        description='Directory to save .pbstream map files',
    )

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

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory',
            FindPackageShare('carto').find('carto') + '/config',
            '-configuration_basename', 'mapping.lua',
        ],
        remappings=[
            ('imu', 'imu/data'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}],
    )

    base_link_to_imu_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    return LaunchDescription([
        map_save_path_arg,
        dm_imu_launch,
        ld06_launch,
        cartographer_node,
        occupancy_grid_node,
        base_link_to_imu_link,
    ])
