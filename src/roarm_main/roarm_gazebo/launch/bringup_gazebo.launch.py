import os
import xacro
import sys
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction

from moveit_configs_utils import MoveItConfigsBuilder

class RoarmMoveItConfig:
    def __init__(self, moveit_config, ros2_controllers):
        self.moveit_config = moveit_config
        self.ros2_controllers = ros2_controllers
        
def get_moveit_config(robot_name: str):
    base_path = f"config/{robot_name}"

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="roarm_moveit")
        #.robot_description(file_path=f"{base_path}/{robot_name}.urdf.xacro")
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
    rviz_config = context.launch_configurations['rviz_config']
    add_camera = context.launch_configurations['add_camera']
    add_depth_camera = context.launch_configurations['add_depth_camera']
    load_controller = LaunchConfiguration('load_controller', default=True)

    share_dir = get_package_share_directory('roarm_moveit')
    roarm_gazebo_dir = get_package_share_directory('roarm_gazebo')
    ROARM_MODEL = os.environ['ROARM_MODEL'] 
    GZ_VERSION = os.environ['GZ_VERSION'] 

    moveit_config = get_moveit_config(ROARM_MODEL)

    xacro_file_name = ROARM_MODEL + '.urdf.xacro'
    xacro_file = os.path.join(
        share_dir,
        'config', 
        ROARM_MODEL,
        xacro_file_name)    
    
    mappings = {
                "use_gazebo": "true",
                "GZ_VERSION": GZ_VERSION,
                "add_camera": add_camera,
                "add_depth_camera": add_depth_camera,
               } 
    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_description = robot_description_config.toxml()
            
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'robot_description': robot_description},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{"use_sim_time": True}],
    )

    controller_nodes = []           
    controller_names = moveit_config.moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    # for controller in controllers:
    if load_controller.perform(context) in ('True', 'true'):
        for controller in controller_names:
            controller_nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller,
                    '--controller-manager', '/controller_manager'
                ],
                parameters=[{'use_sim_time': True}],
            ))

    rviz_parameters = [
        moveit_config.moveit_config.planning_pipelines,
        moveit_config.moveit_config.robot_description_kinematics,
        moveit_config.moveit_config.joint_limits,
        {'use_sim_time': True},
    ]

    # rviz with moveit configuration
    rviz_files= {
        "roarm_description": os.path.join(get_package_share_directory('roarm_description'), 'rviz','view_description.rviz'),
        "roarm_moveit": os.path.join(get_package_share_directory('roarm_moveit'), 'rviz','interact.rviz'),
        "roarm_moveit_servo": os.path.join(get_package_share_directory('roarm_moveit_servo'), 'rviz','interact.rviz'),
        "roarm_moveit_mtc_demo": os.path.join(get_package_share_directory('roarm_moveit_mtc_demo'), 'rviz', 'mtc.rviz'),
    }
    # Get the plugin name based on the use_sim_time argument
    rviz_config_file = rviz_files[rviz_config]

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    world = os.path.join(roarm_gazebo_dir,'worlds','roarm.world')
    models_path = os.path.join(roarm_gazebo_dir, 'models')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    ign_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', ROARM_MODEL,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.903',
        ],
        output='screen'
    )

    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            ]),
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world], 
            'on_exit_shutdown': 'true'
        }.items(),
    )

    ign_gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(roarm_gazebo_dir, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name='create',
        arguments=[
            "-topic", "robot_description",
            "-name", ROARM_MODEL,
            "-robot_namespace", '',
            "-x", '0.0',
            "-y", '0.0',
            '-z', '0.903',
            '-allow_renaming', 'true'
        ],
        output="screen"
    )

    nodes = [        
        robot_state_publisher_node,
    ]

    if GZ_VERSION == 'classic':
        nodes.extend([        
            gazebo_model_path,
            gazebo_server,
            gazebo_client,
            urdf_spawn_node,
        ])

    elif GZ_VERSION == 'harmonic':
        nodes.extend([        
            ign_model_path,
            gzserver_node,
            ign_gazebo_bridge,
            spawn_robot_node,
        ])

    nodes.extend([        
        joint_state_broadcaster_spawner,
        *controller_nodes,
        rviz2_node,
    ])

    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Argument to specify whether to use RViz
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz2'),
        # Argument to specify which RViz configuration to use
        DeclareLaunchArgument('rviz_config', default_value='roarm_description', description='Choose which rviz configuration to use: roarm_description, roarm_moveit, roarm_moveit_servo, roarm_moveit_mtc_demo'),
        DeclareLaunchArgument('add_depth_camera', default_value='false', description='Choose whether to add depth camera'),      
        DeclareLaunchArgument('add_camera', default_value='false', description='Choose whether to add depth camera'),      
        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])
