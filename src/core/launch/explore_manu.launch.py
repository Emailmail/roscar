from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_port = LaunchConfiguration("imu_port")
    chasis_port = LaunchConfiguration("chasis_port")

    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("carto"), "launch", "create_map.launch.py"])
        ]),
        launch_arguments={
            "imu_port": imu_port,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    chasis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("chasis"), "launch", "c30d_keyctrl.launch.py"])
        ]),
        launch_arguments={
            "port": chasis_port,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyAMA4",
                              description="DM IMU serial port"),
        DeclareLaunchArgument("chasis_port", default_value="/dev/ttyACM0",
                              description="C30D chassis serial port (STM32 USART3)"),
        carto_launch,
        chasis_launch,
    ])
