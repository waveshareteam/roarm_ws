import os
import yaml
import sys
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

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

    ROARM_MODEL = os.environ['ROARM_MODEL'] 
    moveit_config = get_moveit_config(ROARM_MODEL)

    share_dir = get_package_share_directory('roarm_moveit')
    xacro_file_name = ROARM_MODEL + '.urdf.xacro'
    xacro_file = os.path.join(
        share_dir,
        'config', 
        ROARM_MODEL,
        xacro_file_name)   

    add_depth_camera = "false"
    use_sim_time = "false"
    for arg in sys.argv:
        if 'use_sim_time' in arg:
            use_sim_time = arg.split(':=')[1]
        if 'add_depth_camera' in arg:
            add_depth_camera = arg.split(':=')[1]

    plugin= {
        "true": "GazeboSystem",
        "True": "GazeboSystem",
        "false": "FakeSystem",
        "False": "FakeSystem",
    }
    # Get the plugin name based on the use_sim_time argument
    ros2_control_plugin = plugin[use_sim_time]

    mappings = {
               "ros2_control_plugin": ros2_control_plugin,
               "add_depth_camera": add_depth_camera,
               } 
    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_description = robot_description_config.toxml()

    rviz_config=os.path.join(get_package_share_directory('roarm_moveit_servo'), 'rviz', 'servo_control.rviz'),

    roarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('roarm_moveit'), 'launch', 'roarm_moveit.launch.py')
        ),
        launch_arguments={
            'rviz_config': rviz_config,
        }.items()
    )
        
    # Get parameters for the Servo node
    servo_yaml = load_yaml("roarm_moveit_servo", "config/roarm_config.yaml")
    servo_yaml['move_group_name'] = "hand"
    roarm_controller = 'hand_controller'
    servo_yaml['command_out_topic'] = '/{}/joint_trajectory'.format(roarm_controller)

    servo_params = {"moveit_servo": servo_yaml}
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "hand"}
    roarm_moveit_servo_container = ComposableNodeContainer(
        name="roarm_moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',
                name='servo_node',
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.moveit_config.robot_description,
                    moveit_config.moveit_config.robot_description_semantic,
                    moveit_config.moveit_config.robot_description_kinematics,
                    moveit_config.moveit_config.joint_limits,
                ],
            ),
            ComposableNode(
                package="roarm_moveit_servo",
                plugin="roarm_moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
        ],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
    )
        
    set_gripper_cmd_node = Node(
        package='roarm_moveit_cmd',
        executable='setgrippercmd',
    )
    
    return LaunchDescription([
        roarm_moveit_launch,
        roarm_moveit_servo_container,
        joy_node,
        set_gripper_cmd_node
    ])

