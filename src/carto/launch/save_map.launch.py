"""Save the current Cartographer map: finish trajectory + write .pbstream to src/map/."""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

MAP_DIR = '/home/relog/roscar/src/map'
PBSTREAM_PATH = os.path.join(MAP_DIR, 'map.pbstream')


def generate_launch_description():
    finish_trajectory = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/finish_trajectory',
            'cartographer_ros_msgs/srv/FinishTrajectory',
            '{trajectory_id: 0}',
        ],
        name='carto_finish_trajectory',
        output='screen',
    )

    write_state = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/write_state',
            'cartographer_ros_msgs/srv/WriteState',
            '{filename: "' + PBSTREAM_PATH + '"}',
        ],
        name='carto_write_state',
        output='screen',
    )

    return LaunchDescription([
        finish_trajectory,
        TimerAction(period=2.0, actions=[write_state]),
    ])
