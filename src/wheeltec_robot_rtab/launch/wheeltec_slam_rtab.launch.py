import os
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', )
    qos = LaunchConfiguration('qos')
    Localization = LaunchConfiguration('Localization')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )

    parameters={
          'frame_id':'camera_link', #wheeltec:camera_link
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    
    remappings=[
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([
        wheeltec_robot,wheeltec_lidar,wheeltec_camera,
        # Set env var to print messages to stdout immediately
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('qos',default_value='2',description='General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('Localization', default_value='false', description='Launch in localization mode.'),        
        # Nodes to launch
        Node(
            package='rtabmap_ros', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.01, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        # Localization mode:
        Node(
            condition=IfCondition(Localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),      
        # SLAM mode:
        Node(
            condition=UnlessCondition(Localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
           
    ])
