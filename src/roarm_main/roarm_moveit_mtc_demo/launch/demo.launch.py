import sys
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
 
    roarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('roarm_moveit'), 'launch', 'roarm_moveit.launch.py')
        ),
        launch_arguments={
            'rviz_config': "roarm_moveit_mtc_demo",
            'capabilities': 'move_group/ExecuteTaskSolutionCapability',
        }.items()
    )

    return LaunchDescription(
        [
            roarm_moveit_launch,
        ]
    )
