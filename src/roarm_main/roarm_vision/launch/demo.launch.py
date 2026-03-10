import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package = "roarm_vision"
    package_shared_path = get_package_share_directory(package)

    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='ugv_roarm_base_link', 
        description='Choose which base_frame'
    )

    cam_frame_arg = DeclareLaunchArgument(
        'cam_frame', default_value='camera_link', 
        description='Choose which cam_frame'
    )

    cam_bringup_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(get_package_share_directory(package), 'launch', 'camera.launch.py')
         ),
    )

    pickplacecmd_node = Node(
        package=package,
        executable='pick_place_cmd',        
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'cam_frame': LaunchConfiguration('cam_frame')
        }]
    )

    node = Node(
        package=package,
        executable=LaunchConfiguration("exe"),
        output="screen",
        parameters=[

        ],
    )

    arg = DeclareLaunchArgument(name="exe")
    return LaunchDescription([
        base_frame_arg,
        cam_frame_arg,
        cam_bringup_launch,
        pickplacecmd_node,
        arg, 
        node
    ])
