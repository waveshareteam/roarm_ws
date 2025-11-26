from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    rviz_config = os.path.join(get_package_share_directory('roarm_moveit_cmd'), 'rviz', 'command_control.rviz')
    
    # Include launch description for bringup_lidar.launch.py
    roarm_moveit_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('roarm_moveit'), 'launch'),
         '/roarm_moveit.launch.py']),
        launch_arguments={
            'rviz_config': rviz_config,
        }.items()
    )
    
    roarm_server_node = Node(
        package='roarm_moveit_cmd',
        executable='roarmserver',
    )
    
    set_gripper_cmd_node = Node(
        package='roarm_moveit_cmd',
        executable='setgrippercmd',
    )   
    
    ld.add_action(roarm_moveit_launch)
    ld.add_action(roarm_server_node) 
    ld.add_action(set_gripper_cmd_node)    
    
    return ld
