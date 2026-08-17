import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Whether to launch RViz2')
    add_camera_arg = DeclareLaunchArgument(
        'add_camera', default_value='false', description='Whether to add hand camera')
    add_depth_camera_arg = DeclareLaunchArgument(
        'add_depth_camera', default_value='false', description='Whether to add depth camera')

    roarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('roarm_moveit'), 'launch', 'roarm_moveit.launch.py')
        ),
        launch_arguments={
            'rviz_config': 'roarm_moveit_mtc_demo',
            'capabilities': 'move_group/ExecuteTaskSolutionCapability',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'add_camera': LaunchConfiguration('add_camera'),
            'add_depth_camera': LaunchConfiguration('add_depth_camera'),
        }.items()
    )

    return LaunchDescription([
        use_rviz_arg,
        add_camera_arg,
        add_depth_camera_arg,
        roarm_moveit_launch,
    ])
