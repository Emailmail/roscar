import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_port = LaunchConfiguration("imu_port")
    map_dir = LaunchConfiguration("map_dir")
    map_name = LaunchConfiguration("map_name")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw_deg = LaunchConfiguration("initial_yaw_deg")
    use_rviz = LaunchConfiguration("use_rviz")

    pkg_share = FindPackageShare("roscar_nav")
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "nav2_view.rviz"])
    map_yaml = PythonExpression(["'", map_dir, "/", map_name, ".yaml'"])

    # ── Localization stack (sensors + Cartographer + pose_to_odom) ──

    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "localize.launch.py"])
        ]),
        launch_arguments={
            "imu_port": imu_port,
            "map_dir": map_dir,
            "map_name": map_name,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw_deg": initial_yaw_deg,
        }.items(),
    )

    # ── Map Server ─────────────────────────────────────────────────

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params, {"yaml_filename": map_yaml}],
    )

    # ── Planner ────────────────────────────────────────────────────

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    # ── Controller ─────────────────────────────────────────────────

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    # ── Behavior Tree Navigator ────────────────────────────────────

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    # ── Behavior Server (recovery actions) ─────────────────────────

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    # ── Lifecycle Manager (auto-start all Nav2 nodes) ──────────────

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[nav2_params],
    )

    # ── RViz ───────────────────────────────────────────────────────

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyAMA4",
                              description="IMU serial port"),
        DeclareLaunchArgument("map_dir", default_value="/home/yilong/roscar/src/map",
                              description="Directory containing map files"),
        DeclareLaunchArgument("map_name", default_value="my_map",
                              description="Map base name (without extension)"),
        DeclareLaunchArgument("initial_x", default_value="0.0",
                              description="Initial robot x in map frame"),
        DeclareLaunchArgument("initial_y", default_value="0.0",
                              description="Initial robot y in map frame"),
        DeclareLaunchArgument("initial_yaw_deg", default_value="0.0",
                              description="Initial robot yaw in degrees"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Launch RViz with Nav2 panel"),
        localize_launch,
        map_server,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
        rviz_node,
    ])
