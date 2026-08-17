from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Whether to launch RViz2')
    add_camera_arg = DeclareLaunchArgument(
        'add_camera', default_value='false', description='Whether to add hand camera')
    add_depth_camera_arg = DeclareLaunchArgument(
        'add_depth_camera', default_value='false', description='Whether to add depth camera')

    roarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('roarm_moveit'), 'launch'),
            '/roarm_moveit.launch.py']),
        launch_arguments={
            'rviz_config': 'roarm_moveit_cmd',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'add_camera': LaunchConfiguration('add_camera'),
            'add_depth_camera': LaunchConfiguration('add_depth_camera'),
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

    ld.add_action(use_rviz_arg)
    ld.add_action(add_camera_arg)
    ld.add_action(add_depth_camera_arg)
    ld.add_action(roarm_moveit_launch)
    ld.add_action(roarm_server_node)
    ld.add_action(set_gripper_cmd_node)

    return ld
