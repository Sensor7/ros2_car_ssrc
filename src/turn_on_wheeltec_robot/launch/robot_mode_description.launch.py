import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_robot_node(robot_urdf,child):
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{child}',
        arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'), 'urdf', robot_urdf)],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )
    
def generate_launch_description():
    #akm
    senior_akm = LaunchConfiguration('senior_akm', default='false')
    top_akm_bs = LaunchConfiguration('top_akm_bs', default='false')
    top_akm_dl = LaunchConfiguration('top_akm_dl', default='false')
    #mec
    senior_mec_bs = LaunchConfiguration('senior_mec_bs', default='false')
    senior_mec_dl = LaunchConfiguration('senior_mec_dl', default='false')
    top_mec_bs = LaunchConfiguration('top_mec_bs', default='false')
    top_mec_dl = LaunchConfiguration('top_mec_dl', default='false')
    mec_EightDrive_robot = LaunchConfiguration('mec_EightDrive_robot', default='false')
    flagship_mec_bs_robot = LaunchConfiguration('flagship_mec_bs_robot', default='false')
    flagship_mec_dl_robot = LaunchConfiguration('flagship_mec_dl_robot', default='false')
    #omni
    senior_omni = LaunchConfiguration('senior_omni', default='false')
    top_omni = LaunchConfiguration('top_omni', default='false')
    #4wd
    senior_4wd_bs_robot = LaunchConfiguration('senior_4wd_bs_robot', default='false')
    senior_4wd_dl_robot = LaunchConfiguration('senior_4wd_dl_robot', default='false')
    flagship_4wd_bs_robot = LaunchConfiguration('flagship_4wd_bs_robot', default='false')
    flagship_4wd_dl_robot = LaunchConfiguration('flagship_4wd_dl_robot', default='false')
    top_4wd_bs_robot = LaunchConfiguration('top_4wd_bs_robot', default='false')
    top_4wd_dl_robot = LaunchConfiguration('top_4wd_dl_robot', default='false')
    #diff*5
    senior_diff_robot = LaunchConfiguration('senior_diff_robot', default='false')
    four_wheel_diff_bs = LaunchConfiguration('four_wheel_diff_bs', default='false')
    four_wheel_diff_dl = LaunchConfiguration('four_wheel_diff_dl', default='false')
    flagship_four_wheel_diff_bs_robot = LaunchConfiguration('flagship_four_wheel_diff_bs_robot', default='false')
    flagship_four_wheel_diff_dl_robot = LaunchConfiguration('flagship_four_wheel_diff_dl_robot', default='false')

    #akm¡Á3 lidar is ls_lidar,change the parameter from 3.14 to 0
    senior_akm_ = GroupAction(
        condition=IfCondition(senior_akm),
        actions=[
        generate_robot_node('senior_akm_robot.urdf','senior_akm'),
        generate_static_transform_publisher_node(['0.26 ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.26 ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    top_akm_bs_ = GroupAction(
        condition=IfCondition(top_akm_bs),
        actions=[
        generate_robot_node('top_akm_bs_robot.urdf','top_akm_bs'),
        generate_static_transform_publisher_node(['0.53 ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.53', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    top_akm_dl_ = GroupAction(
        condition=IfCondition(top_akm_dl),
        actions=[
        generate_robot_node('top_akm_dl_robot.urdf','top_akm_dl'),
        generate_static_transform_publisher_node(['0.497 ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.58', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ])  
    #mec¡Á7 lidar is ls_lidar,change the parameter from 3.14 to 0
    senior_mec_bs_ = GroupAction(
        condition=IfCondition(senior_mec_bs),
        actions=[
        generate_robot_node('senior_mec_robot.urdf','senior_mec_bs'),
        generate_static_transform_publisher_node(['0.1 ', '0', '0.165'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.18', '0', '0.3'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    senior_mec_dl_ = GroupAction(
        condition=IfCondition(senior_mec_dl),
        actions=[
        generate_robot_node('senior_mec_dl_robot.urdf','senior_mec_dl'),
        generate_static_transform_publisher_node(['0.165 ', '0', '0.235'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.255', '0', '0.35'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    top_mec_bs_ = GroupAction(
        condition=IfCondition(top_mec_bs),
        actions=[
        generate_robot_node('top_mec_bs_robot.urdf','top_mec_bs'),
        generate_static_transform_publisher_node(['0.155 ', '0', '0.195'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.24', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    top_mec_dl_ = GroupAction(
        condition=IfCondition(top_mec_bs),
        actions=[
        generate_robot_node('top_mec_dl_robot.urdf','top_mec_dl'),
        generate_static_transform_publisher_node(['0.155 ', '0', '0.195'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.24', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ])     
    mec_EightDrive_robot_ = GroupAction(
        condition=IfCondition(mec_EightDrive_robot),
        actions=[
        generate_robot_node('mec_EightDrive_robot.urdf','mec_EightDrive_robot'),
        generate_static_transform_publisher_node(['0.207 ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.32', '0', '0.2'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    flagship_mec_bs_robot_ = GroupAction(
        condition=IfCondition(flagship_mec_bs_robot),
        actions=[
        generate_robot_node('flagship_mec_bs_robot.urdf','flagship_mec_bs'),
        generate_static_transform_publisher_node(['0.267  ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.32', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    flagship_mec_dl_robot_ = GroupAction(
        condition=IfCondition(flagship_mec_dl_robot),
        actions=[
        generate_robot_node('flagship_mec_dl_robot.urdf','flagship_mec_dl'),
        generate_static_transform_publisher_node(['0.267  ', '0', '0.228'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.32', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    # add omni*2, lidar is ls_lidar,change the parameter from 3.14 to 0
    senior_omni_ = GroupAction(
        condition=IfCondition(senior_omni),
        actions=[
        generate_robot_node('senior_omni_robot.urdf','senior_omni'),
        generate_static_transform_publisher_node(['0.087  ', '0', '0.23'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.187', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    top_omni_ = GroupAction(
        condition=IfCondition(top_omni),
        actions=[
        generate_robot_node('top_omni_robot.urdf','top_akm_bs'),
        generate_static_transform_publisher_node(['0.149  ', '0', '0.23'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.25', '0', '0.32'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 

    #
    #4wd¡Á6 lidar is ls_lidar,change the parameter from 3.14 to 0
    senior_4wd_bs_robot_ = GroupAction(
        condition=IfCondition(senior_4wd_bs_robot),
        actions=[
        generate_robot_node('senior_4wd_bs_robot.urdf','senior_4wd_bs'),
        generate_static_transform_publisher_node(['0.02', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.14', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    senior_4wd_dl_robot_ = GroupAction(
        condition=IfCondition(senior_mec_dl),
        actions=[
        generate_robot_node('senior_4wd_dl_robot.urdf','senior_4wd_dl'),
        generate_static_transform_publisher_node(['0.02', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.14', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    flagship_4wd_bs_robot_ = GroupAction(
        condition=IfCondition(flagship_4wd_bs_robot),
        actions=[
        generate_robot_node('flagship_4wd_bs_robot.urdf','flagship_4wd_bs'),
        generate_static_transform_publisher_node(['0.02', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.14', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    flagship_4wd_dl_robot_ = GroupAction(
        condition=IfCondition(top_mec_bs),
        actions=[
        generate_robot_node('flagship_4wd_dl_robot.urdf','flagship_4wd_dl'),
        generate_static_transform_publisher_node(['0.02', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.14', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ])     
    top_4wd_bs_robot_ = GroupAction(
        condition=IfCondition(top_4wd_bs_robot),
        actions=[
        generate_robot_node('top_4wd_bs_robot.urdf','top_4wd_bs'),
        generate_static_transform_publisher_node(['0.02', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.14', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    top_4wd_dl_robot_ = GroupAction(
        condition=IfCondition(top_4wd_dl_robot),
        actions=[
        generate_robot_node('top_4wd_dl_robot.urdf','top_4wd_dl'),
        generate_static_transform_publisher_node(['0.02', '0', '0.155'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.14', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    
    
    #diff¡Á5 lidar is ls_lidar,change the parameter from 3.14 to 0
    senior_diff_robot_ = GroupAction(
        condition=IfCondition(senior_4wd_bs_robot),
        actions=[
        generate_robot_node('senior_diff_robot.urdf','senior_diff'),
        generate_static_transform_publisher_node(['0.12', '0', '0.195'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.125', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    four_wheel_diff_bs_ = GroupAction(
        condition=IfCondition(senior_mec_dl),
        actions=[
        generate_robot_node('four_wheel_diff_bs_robot.urdf','four_wheel_diff_bs'),
        generate_static_transform_publisher_node(['0.157', '0', '0.257'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.157', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    four_wheel_diff_dl_ = GroupAction(
        condition=IfCondition(four_wheel_diff_dl),
        actions=[
        generate_robot_node('four_wheel_diff_dl_robot.urdf','four_wheel_diff_dl'),
        generate_static_transform_publisher_node(['0.272', '0', '0.257'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.272', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    flagship_four_wheel_diff_bs_robot_ = GroupAction(
        condition=IfCondition(flagship_four_wheel_diff_bs_robot),
        actions=[
        generate_robot_node('flagship_four_wheel_diff_bs_robot.urdf','flagship_4wheel_diff_bs'),
        generate_static_transform_publisher_node(['0.272', '0', '0.257'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.272', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ])     
    flagship_four_wheel_diff_dl_robot_ = GroupAction(
        condition=IfCondition(flagship_four_wheel_diff_dl_robot),
        actions=[
        generate_robot_node('flagship_four_wheel_diff_dl_robot.urdf','flagship_4wheel_diff_dl'),
        generate_static_transform_publisher_node(['0.272', '0', '0.257'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.272', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 

     
    # Create the launch description and populate
    ld = LaunchDescription()
    #add akm*3
    ld.add_action(senior_akm_)
    ld.add_action(top_akm_bs_)
    ld.add_action(top_akm_dl_)
    #add mec*7
    ld.add_action(senior_mec_bs_)
    ld.add_action(senior_mec_dl_)
    ld.add_action(top_mec_bs_)
    ld.add_action(top_mec_dl_)
    ld.add_action(mec_EightDrive_robot_)
    ld.add_action(flagship_mec_bs_robot_)
    ld.add_action(flagship_mec_dl_robot_) 
    #add omni*3
    ld.add_action(senior_omni_)
    ld.add_action(top_omni_)
    #add 4wd*6
    ld.add_action(senior_4wd_bs_robot_)
    ld.add_action(senior_4wd_dl_robot_)
    ld.add_action(flagship_4wd_bs_robot_)
    ld.add_action(flagship_4wd_dl_robot_)
    ld.add_action(top_4wd_bs_robot_)
    ld.add_action(top_4wd_dl_robot_) 
    #add diff*5
    ld.add_action(senior_diff_robot_)
    ld.add_action(four_wheel_diff_bs_)
    ld.add_action(four_wheel_diff_dl_)
    ld.add_action(flagship_four_wheel_diff_bs_robot_)
    ld.add_action(flagship_four_wheel_diff_dl_robot_)
    return ld
