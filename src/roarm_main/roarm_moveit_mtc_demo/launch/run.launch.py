from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import sys
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
    
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    ROARM_MODEL = os.environ['ROARM_MODEL'] 
    moveit_config = get_moveit_config(ROARM_MODEL)
    
    share_dir = get_package_share_directory('roarm_moveit')
    xacro_file_name = ROARM_MODEL + '.urdf.xacro'
    xacro_file = os.path.join(
        share_dir,
        'config', 
        ROARM_MODEL,
        xacro_file_name)    
    
    use_sim_time_str = "false"
    add_depth_camera = "false"
    for arg in sys.argv:
        if 'use_sim_time' in arg:
            use_sim_time_str = arg.split(':=')[1]
        if 'add_depth_camera' in arg:
            add_depth_camera = arg.split(':=')[1]
    plugin= {
        "true": "GazeboSystem",
        "True": "GazeboSystem",
        "false": "FakeSystem",
        "False": "FakeSystem",
    }
    # Get the plugin name based on the use_sim_time argument
    ros2_control_plugin = plugin[use_sim_time_str]

    mappings = {
               "ros2_control_plugin": ros2_control_plugin,    
               "add_depth_camera": add_depth_camera,
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
    return LaunchDescription([arg, node])
