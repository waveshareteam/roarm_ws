import os
import sys
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from srdfdom.srdf import SRDF
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from ament_index_python.packages import get_package_share_directory 
from moveit_configs_utils import MoveItConfigsBuilder

def str_to_bool(s):
    return s.lower() in ['true', 'True']

class RoarmMoveItConfig:
    def __init__(self, moveit_config, ros2_controllers):
        self.moveit_config = moveit_config
        self.ros2_controllers = ros2_controllers
        
def get_moveit_config(robot_name: str):
    base_path = f"config/{robot_name}"

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="roarm_moveit")
        # .robot_description(file_path=f"{base_path}/{robot_name}.urdf.xacro")
        .robot_description_semantic(file_path=f"{base_path}/{robot_name}.srdf")
        .robot_description_kinematics(file_path=f"{base_path}/kinematics.yaml")
        .trajectory_execution(file_path=f"{base_path}/moveit_controllers.yaml")
        .sensors_3d(file_path=f"{base_path}/sensors_3d.yaml")
        .joint_limits(file_path=f"{base_path}/joint_limits.yaml")
        .pilz_cartesian_limits(file_path=f"{base_path}/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    share_dir = get_package_share_directory('roarm_moveit')
    ros2_controllers = os.path.join(share_dir, f"{base_path}/ros2_controllers.yaml")  
    moveit_config = RoarmMoveItConfig(moveit_config, ros2_controllers)

    return moveit_config

# Function to set up and launch ROS 2 nodes based on the given context
def launch_setup(context, *args, **kwargs):
    rviz_config = context.launch_configurations['rviz_config']
    add_camera = context.launch_configurations['add_camera']
    add_depth_camera = context.launch_configurations['add_depth_camera']
    
    share_dir = get_package_share_directory('roarm_moveit')
    ROARM_MODEL = os.environ['ROARM_MODEL'] 
    GRIPPER_TYPE = os.environ['GRIPPER_TYPE']
  
    xacro_file_name = ROARM_MODEL + '.urdf.xacro'
    xacro_file = os.path.join(
        share_dir,
        'config', 
        ROARM_MODEL,
        xacro_file_name)   

    mappings = {
                "use_gazebo": "false",
                "add_depth_camera": add_depth_camera,
                "add_camera": add_camera,
                "gripper_type": GRIPPER_TYPE,
               } 
    
    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_description = robot_description_config.toxml()

    moveit_config = get_moveit_config(ROARM_MODEL)

    # Given the published joint states, publish tf for the robot links and the robot description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {'robot_description': robot_description},
        ],
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_geometry_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_state_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "publish_transforms_updates": LaunchConfiguration("publish_monitored_planning_scene"),
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.moveit_config.to_dict(),
        move_group_configuration, 
        robot_description,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.ros2_controllers
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    controller_nodes = [] 
    controller_names = moveit_config.moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    for controller in controller_names + ["joint_state_broadcaster"]:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller],
        ))

    rviz_files= {
        "roarm_description": os.path.join(get_package_share_directory('roarm_description'), 'rviz','view_description.rviz'),
        "roarm_moveit": os.path.join(get_package_share_directory('roarm_moveit'), 'rviz','interact.rviz'),
        "roarm_moveit_cmd": os.path.join(get_package_share_directory('roarm_moveit_cmd'), 'rviz', 'command_control.rviz'),
        "roarm_moveit_servo": os.path.join(get_package_share_directory('roarm_moveit_servo'), 'rviz','servo_control.rviz'),
        "roarm_moveit_mtc_demo": os.path.join(get_package_share_directory('roarm_moveit_mtc_demo'), 'rviz', 'mtc.rviz'),
    }
    # Get the plugin name based on the use_sim_time argument
    rviz_config_file = rviz_files[rviz_config]

    rviz_parameters = [
        moveit_config.moveit_config.planning_pipelines,
        moveit_config.moveit_config.robot_description_kinematics,
        moveit_config.moveit_config.joint_limits,
    ]

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config_file],
        parameters=rviz_parameters,
        output='screen'
    )

    return [
        robot_state_publisher_node,
        move_group_node,
        ros2_control_node,
        *controller_nodes,
        rviz2_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        # Argument to specify whether to use RViz
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz2'),
        # Argument to specify which RViz configuration to use
        DeclareLaunchArgument('rviz_config', default_value='roarm_moveit', description='Choose which rviz configuration to use: description, bringup, moveit, moveit_servo, moveit_mtc, slam_2d, slam_3d, nav_2d, nav_3d'),
        DeclareLaunchArgument('add_camera', default_value='false', description='Choose whether to add camera'),   
        DeclareLaunchArgument('add_depth_camera', default_value='false', description='Choose whether to add depth camera'),   
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True),
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True),
        DeclareLaunchArgument("capabilities",default_value=""),
        DeclareLaunchArgument("disable_capabilities",default_value=""),
        DeclareBooleanLaunchArg("monitor_dynamics", default_value=False),
        DeclareLaunchArgument("publish_frequency", default_value="15.0"),

        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])