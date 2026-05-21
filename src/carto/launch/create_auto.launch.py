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
    nav2_params = PathJoinSubstitution(
        [FindPackageShare("carto"), "config", "nav2_slam_params.yaml"]
    )

    # ── Sensors ────────────────────────────────────────────────────

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

    # ── TF: base_link → imu_link (identity) ───────────────────────

    base_link_to_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_link",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"],
    )

    # ── Cartographer (mapping mode) ─────────────────────────────────

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", carto_config_dir,
            "-configuration_basename", "cartographer_2d_auto.lua",
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

    # ── Pose to Odometry (Nav2 needs /odom topic) ──────────────────

    pose_to_odom = Node(
        package="roscar_nav",
        executable="pose_to_odom",
        name="pose_to_odom",
        output="screen",
    )

    # ── Nav2 (no map_server — map comes from Cartographer) ──────────

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[nav2_params],
    )

    # ── Exploration (explore_lite — frontier-based) ──────────────────

    explore_lite_params = PathJoinSubstitution(
        [FindPackageShare("carto"), "config", "explore_lite_params.yaml"]
    )

    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[explore_lite_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation clock"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyAMA4",
                              description="DM IMU serial port"),
        imu_launch,
        lidar_launch,
        base_link_to_imu_tf,
        cartographer_node,
        occupancy_grid_node,
        pose_to_odom,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
        explore_node,
    ])
