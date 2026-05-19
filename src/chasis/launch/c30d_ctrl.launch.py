from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    publish_hz = LaunchConfiguration('publish_hz')

    c30d_node = Node(
        package='chasis',
        executable='c30d_ctrl_node',
        name='c30d_ctrl',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate,
            'publish_hz': publish_hz,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyACM0',
            description='Serial port for C30D chassis controller (STM32 USART3)'
        ),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='Serial baudrate (115200, 8N1)'
        ),
        DeclareLaunchArgument(
            'publish_hz', default_value='50.0',
            description='Control frame send rate (Hz, 20-50 recommended)'
        ),
        c30d_node,
    ])
