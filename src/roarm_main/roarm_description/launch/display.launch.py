import xacro
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# Function to set up and launch ROS 2 nodes based on the given context
def launch_setup(context, *args, **kwargs):
    add_camera = context.launch_configurations['add_camera']
    add_depth_camera = context.launch_configurations['add_depth_camera']
    
    share_dir = get_package_share_directory('roarm_description')
    ROARM_MODEL = os.environ['ROARM_MODEL']
    GRIPPER_TYPE = os.environ['GRIPPER_TYPE']

    xacro_file_name = ROARM_MODEL + '.xacro'
    xacro_file = os.path.join(
        share_dir,
        'urdf/bases',   
        xacro_file_name)    

    mappings = {
                "use_gazebo": "false",
                "add_depth_camera": add_depth_camera,
                "add_camera": add_camera,
                "gripper_type": GRIPPER_TYPE,
               } 

    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_description = robot_description_config.toxml()

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_config_file = os.path.join(share_dir, 'rviz', 'view_description.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return [
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]

def generate_launch_description():
        return LaunchDescription([
        # Argument to specify whether to use RViz
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz2'),
        DeclareLaunchArgument('add_camera', default_value='false', description='Choose whether to add camera'),   
        DeclareLaunchArgument('add_depth_camera', default_value='false', description='Choose whether to add depth camera'),   
        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])