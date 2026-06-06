import os
import yaml
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

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

def launch_setup(context, *args, **kwargs):
    add_camera = context.launch_configurations['add_camera']
    add_depth_camera = context.launch_configurations['add_depth_camera']
    
    share_dir = get_package_share_directory('roarm_moveit')
    ROARM_MODEL = os.environ['ROARM_MODEL'] 
    GRIPPER_TYPE = os.environ['GRIPPER_TYPE']

    moveit_config = get_moveit_config(ROARM_MODEL)
    
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
    
    package = "roarm_moveit_mtc_demo"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable=LaunchConfiguration("exe"),
        output="screen",
        parameters=[
            #moveit_config.moveit_config.robot_description,
            {'robot_description': robot_description},
            moveit_config.moveit_config.robot_description_semantic,
            moveit_config.moveit_config.robot_description_kinematics,
            moveit_config.moveit_config.joint_limits,
            moveit_config.moveit_config.planning_pipelines,
            os.path.join(package_shared_path, "config", "roarm_config.yaml"),
        ],
    )

    arg = DeclareLaunchArgument(name="exe")
    return [arg, node]

# Function to generate the launch description with configurable arguments
def generate_launch_description():
    return LaunchDescription([
        # Argument to specify whether to use RViz
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz2'),
        DeclareLaunchArgument('add_camera', default_value='false', description='Choose whether to add camera'),      
        DeclareLaunchArgument('add_depth_camera', default_value='false', description='Choose whether to add depth camera'),      
        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])