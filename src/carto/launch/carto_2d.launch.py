from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_laser_z = LaunchConfiguration("base_laser_z")
    imu_z = LaunchConfiguration("imu_z")
    publish_base_laser_tf = LaunchConfiguration("publish_base_laser_tf")
    publish_imu_tf = LaunchConfiguration("publish_imu_tf")

    carto_config_dir = PathJoinSubstitution([FindPackageShare("carto"), "config"])
    carto_config_basename = "cartographer_2d.lua"

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("base_laser_z", default_value="0.18"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("publish_base_laser_tf", default_value="false"),
        DeclareLaunchArgument("publish_imu_tf", default_value="true"),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_base_laser",
            condition=IfCondition(publish_base_laser_tf),
            arguments=["--x", "0.0", "--y", "0.0", "--z", base_laser_z,
                       "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                       "--frame-id", "base_link", "--child-frame-id", "base_laser"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu_link",
            condition=IfCondition(publish_imu_tf),
            arguments=["--x", "0.0", "--y", "0.0", "--z", imu_z,
                       "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        ),
        Node(
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
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
