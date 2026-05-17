from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_port = LaunchConfiguration("imu_port")

    carto_config_dir = PathJoinSubstitution([FindPackageShare("carto"), "config"])
    carto_config_basename = "cartographer_2d.lua"

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("dm_imu"), "launch", "dm_imu.launch.py"])
        ]),
        launch_arguments={"port": imu_port}.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ldlidar_stl_ros2"), "launch", "ld06.launch.py"])
        ]),
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", carto_config_dir,
            "-configuration_basename", carto_config_basename,
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyACM1"),
        imu_launch,
        lidar_launch,
        cartographer_node,
        occupancy_grid_node,
    ])
