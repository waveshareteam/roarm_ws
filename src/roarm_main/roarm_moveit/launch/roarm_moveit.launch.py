import os
import sys
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

def generate_launch_description():
    ld = LaunchDescription()
    ros_namespace = LaunchConfiguration('ros_namespace', default='')

    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='False'
    )
    ld.add_action(use_rviz_arg)

    use_rviz= LaunchConfiguration("use_rviz")
    
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

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    name_counter = 0

    for key, xml_contents in moveit_config.moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1

    # Given the published joint states, publish tf for the robot links
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        condition=UnlessCondition(use_sim_time),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {'robot_description': robot_description},
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    
    ld.add_action(rsp_node)

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value="",
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value="",
        )
    )

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        {'robot_description': robot_description},
        moveit_config.moveit_config.to_dict(),
        move_group_configuration,
        {'use_sim_time': str_to_bool(use_sim_time)}, 
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ["DISPLAY"]},
    )
    
    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.moveit_config.package_path / "rviz/interact.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.moveit_config.planning_pipelines,
        moveit_config.moveit_config.robot_description_kinematics,
        moveit_config.moveit_config.joint_limits,
        {'use_sim_time': str_to_bool(use_sim_time)}, 
    ]

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        output='screen'
    )
    ld.add_action(rviz_node)

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("reset", default_value=False))

    # The default DB port for moveit (not default MongoDB port to avoid potential conflicts)
    ld.add_action(DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"))

    # The default DB host for moveit
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost")
    )

    # Load warehouse parameters
    db_parameters = [
        {
            "overwrite": False,
            "database_path": LaunchConfiguration("moveit_warehouse_database_path"),
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            "warehouse_exec": "mongod",
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        },
    ]
    # Run the DB server
    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        # TODO(dlu): Figure out if this needs to be run in a specific directory
        # (ROS 1 version set cwd="ROS_HOME")
        parameters=db_parameters,
    )
    #ld.add_action(db_node)

    # If we want to reset the database, run this node
    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset")),
    )
    #ld.add_action(reset_node)
    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.ros2_controllers
            ],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
        )
    )

    controller_names = moveit_config.moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                condition=UnlessCondition(use_sim_time),
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                ],
                output="screen",
            )
        )

    return ld
