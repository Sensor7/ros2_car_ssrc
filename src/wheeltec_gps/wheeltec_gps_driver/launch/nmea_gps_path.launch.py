from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('wheeltec_gps_driver')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_gps = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_nmea_driver.launch.py')),
    )

    return LaunchDescription([
        wheeltec_gps,
        launch_ros.actions.Node(
            package='wheeltec_gps_path', executable='gps_path', output='screen',
            remappings=[('/gps_topic','/gps/fix')]),
            
    ])
