import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

    carto_config_dir = PathJoinSubstitution([FindPackageShare("roscar_nav"), "config"])
    map_path = PathJoinSubstitution([map_dir, map_name])
    pbstream_path = PythonExpression(["'", map_path, ".pbstream'"])

    # ── Sensors ────────────────────────────────────────────────────

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("dm_imu"), "launch", "dm_imu.launch.py"])
        ]),
        launch_arguments={"port": imu_port}.items(),
    )

    # Note: lidar port is hardcoded in ld06.launch.py (/dev/ttyUSB0)
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

    # ── Cartographer pure localization ─────────────────────────────

    def make_cartographer_node(context):
        cfg_dir = carto_config_dir.perform(context)
        pbstream = pbstream_path.perform(context)

        args = [
            "-configuration_directory", cfg_dir,
            "-configuration_basename", "carto_localize.lua",
            "-load_state_filename", pbstream,
        ]

        # Only pass initial pose when user explicitly sets non-zero values.
        x = float(context.launch_configurations["initial_x"])
        y = float(context.launch_configurations["initial_y"])
        yaw_deg = float(context.launch_configurations["initial_yaw_deg"])

        if x != 0.0 or y != 0.0 or yaw_deg != 0.0:
            yaw_rad = math.radians(yaw_deg)
            pose_str = (
                "{to_trajectory_id = 0,"
                f" relative_pose = {{ translation = {{ {x}, {y}, 0. }},"
                f" rotation = {{ 0., 0.,"
                f" {math.sin(yaw_rad / 2.0)}, {math.cos(yaw_rad / 2.0)} }}}}}}"
            )
            args.extend(["-initial_trajectory_pose", pose_str])

        return [Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": False}],
            arguments=args,
            remappings=[
                ("scan", "/scan"),
                ("imu", "/imu/data"),
            ],
        )]

    # ── Pose to Odometry ───────────────────────────────────────────

    pose_to_odom = Node(
        package="roscar_nav",
        executable="pose_to_odom",
        name="pose_to_odom",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0",
                              description="IMU serial port"),
        DeclareLaunchArgument("map_dir", default_value="/home/yilong/roscar/src/map",
                              description="Directory containing map files"),
        DeclareLaunchArgument("map_name", default_value="my_map",
                              description="Map base name (without extension)"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw_deg", default_value="0.0"),
        imu_launch,
        lidar_launch,
        base_link_to_imu_tf,
        OpaqueFunction(function=make_cartographer_node),
        pose_to_odom,
    ])
