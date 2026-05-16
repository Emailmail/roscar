"""Full autonomous navigation — localization + Nav2 path planning."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

MAP_DIR = '/home/relog/roscar/src/map'


def generate_launch_description():
    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename',
        default_value=os.path.join(MAP_DIR, 'map.pbstream'),
        description='Path to .pbstream map for localization',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav'), 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'load_state_filename': LaunchConfiguration('load_state_filename'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav'), 'launch', 'navigation.launch.py')
        ),
    )

    return LaunchDescription([
        load_state_filename_arg,
        localization_launch,
        navigation_launch,
    ])
