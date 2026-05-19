from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    map_dir = LaunchConfiguration("map_dir")

    save_all = ExecuteProcess(
        cmd=["bash", "-c",
             "ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "
             "\"{filename: '$1/$2.pbstream'}\" && "
             "ros2 run nav2_map_server map_saver_cli -f \"$1/$2\" && "
             "echo Saved: $1/$2.pgm + $1/$2.yaml + $1/$2.pbstream",
             "bash", map_dir, map_name],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="my_map",
                              description="Map file name (without extension)"),
        DeclareLaunchArgument("map_dir", default_value="/home/yilong/roscar/src/map",
                              description="Directory to save maps"),
        save_all,
    ])
