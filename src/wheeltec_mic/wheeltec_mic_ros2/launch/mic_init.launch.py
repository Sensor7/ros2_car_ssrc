from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wheeltec_mic = Node(
        package="wheeltec_mic_ros2",
        executable="wheeltec_mic",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}]
    )

    voice_control = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        output='screen',
        parameters=[{"appid": "6159904a"}]
    )

    ld = LaunchDescription()

    ld.add_action(wheeltec_mic)
    ld.add_action(voice_control)
    
    return ld