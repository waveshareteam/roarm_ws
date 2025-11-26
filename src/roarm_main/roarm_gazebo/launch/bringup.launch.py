#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import xacro
import sys
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction

from launch_ros.parameter_descriptions import ParameterValue
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

    use_rviz = LaunchConfiguration('use_rviz', default=False)
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui', default=True)
  
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
    for arg in sys.argv:
        if 'add_depth_camera' in arg:
            add_depth_camera = arg.split(':=')[1]
        if 'rviz_config' in arg:
            rviz_config = arg.split(':=')[1]

    mappings = {
               "ros2_control_plugin": "GazeboSystem",
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
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
       
    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    roarm_gazebo_world = PathJoinSubstitution([FindPackageShare('roarm_gazebo'), 'worlds', 'table.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': roarm_gazebo_world,
            'server': 'true',
            'gui': use_gazebo_gui,
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', ROARM_MODEL,
            '-x', '0.0',
            '-y', '-0.54',
            '-z', '1.021',
            '-Y', '-1.5708',
        ],
        parameters=[{'use_sim_time': True}],
    )

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
    rviz_config_path = rviz_files[rviz_config]

    #rviz_config = os.path.join(get_package_share_directory('roarm_moveit'), "rviz/interact.rviz")
    # rviz_config = os.path.join(get_package_share_directory('roarm_moveit_mtc_demo'), 'rviz', 'mtc.rviz'),
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=rviz_parameters,
        condition=IfCondition(use_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
    
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue("", value_type=str),
        "disable_capabilities": ParameterValue("", value_type=str),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.moveit_config.to_dict(),
        move_group_configuration,
        {'use_sim_time': True},
        move_group_capabilities,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '{}/controller_manager'.format('')],
        output='screen',
    )

    controller_nodes = []           
    controller_names = moveit_config.moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
             
    # for controller in controllers:
    for controller in controller_names:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format('')
            ],
            parameters=[{'use_sim_time': True}],
        ))
            
    return [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner_node,
                on_exit=controller_nodes,
            )
        ),        
        gazebo_launch,
        gazebo_spawn_entity_node,
        rviz2_node,        
        move_group_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
