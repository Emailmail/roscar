from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_port = LaunchConfiguration("imu_port")
    chasis_port = LaunchConfiguration("chasis_port")
    map_name = LaunchConfiguration("map_name")

    # ── Chassis control ──────────────────────────────────────────────

    chasis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("chasis"), "launch", "c30d_ctrl.launch.py"])
        ]),
        launch_arguments={"port": chasis_port}.items(),
    )

    # ── Navigation (localization + Nav2) ─────────────────────────────

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("roscar_nav"), "launch", "navigate.launch.py"])
        ]),
        launch_arguments={
            "imu_port": imu_port,
            "map_name": map_name,
            "use_rviz": "false",
        }.items(),
    )

    # ── Waypoint follower ────────────────────────────────────────────

    waypoint_follower = Node(
        package="core",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyAMA4",
                              description="IMU serial port"),
        DeclareLaunchArgument("chasis_port", default_value="/dev/ttyACM0",
                              description="C30D chassis serial port (STM32 USART3)"),
        DeclareLaunchArgument("map_name", default_value="my_map",
                              description="Map base name (without extension)"),
        chasis_launch,
        nav_launch,
        waypoint_follower,
    ])
