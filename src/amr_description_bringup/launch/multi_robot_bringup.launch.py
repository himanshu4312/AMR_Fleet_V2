#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

ROBOTS = [
    {'name': 'robot1', 'x': '-12.2', 'y': '9.95', 'yaw': '0.0'},
    {'name': 'robot2', 'x': '2.65', 'y': '-13.3', 'yaw': '0.0'},
    {'name': 'robot3', 'x': '3.90', 'y': '5.25', 'yaw': '0.0'},
    {'name': 'robot4', 'x': '3.90', 'y': '-5.85', 'yaw': '0.0'},
]

WORLD_NAME = 'empty'
ROBOT_STAGGER_DELAY_S = 6.0


def make_robot_group(robot, urdf_path, nav2_params_path, pkg_nav2_bringup, use_sim_time):
    name = robot['name']
    x, y, yaw = robot['x'], robot['y'], robot['yaw']

    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=', name]),
        value_type=str,
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        remappings=tf_remap,
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=name,
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', name,
            '-x', x,
            '-y', y,
            '-z', '0.0',
            '-Y', yaw,
        ],
    )

    gz_joint_state_topic = f'/world/{WORLD_NAME}/model/{name}/joint_state'
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'{gz_joint_state_topic}@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[(gz_joint_state_topic, f'/{name}/joint_states')],
    )

    param_rewrites = {
        'base_frame_id': f'{name}/base_link',
        'odom_frame_id': f'{name}/odom',
        'scan_topic': f'/{name}/scan',
        'robot_base_frame': f'{name}/base_link',
        'odom_topic': f'/{name}/odom',
        'topic': f'/{name}/scan',
        'local_costmap.local_costmap.ros__parameters.global_frame': f'{name}/odom',
        'behavior_server.ros__parameters.global_frame': f'{name}/odom',
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': '/map',
    }

    robot_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key=name,
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=name,
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            ParameterFile(robot_params, allow_substs=True),
            {
                'use_sim_time': use_sim_time,
                'set_initial_pose': True,
                'initial_pose.x': float(x),
                'initial_pose.y': float(y),
                'initial_pose.z': 0.0,
                'initial_pose.yaw': float(yaw),
            },
        ],

        remappings=tf_remap + [('map', '/map')],
    )

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
        }],
    )

    navigation_group = GroupAction([
        PushRosNamespace(name),
        SetRemap(src='map', dst='/map'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),

            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': robot_params,
                'use_composition': 'False',
                'container_name': 'nav2_container',
                'use_respawn': 'true',
            }.items(),
        ),
    ])

    return GroupAction([
        rsp_node,
        spawn_node,
        bridge_node,
        amcl_node,
        lifecycle_manager_localization_node,
        navigation_group,
    ])


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_description_bringup = get_package_share_directory('amr_description_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_path = os.path.join(pkg_amr_description, 'urdf', 'amr_body.urdf.xacro.xml')
    world_path = os.path.join(pkg_amr_description, 'world', 'maze.sdf')
    map_yaml_path = os.path.join(pkg_amr_description_bringup, 'maps', 'maze_map_v3.yaml')
    nav2_params_path = os.path.join(pkg_amr_description_bringup, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    headless = LaunchConfiguration('headless')
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description=(
            'Run Gazebo with -s --headless-rendering (server only, offscreen '
            'sensor rendering, no GUI window). Two robots each running a full '
            'Nav2 stack plus GPU lidar rendering can be heavy on constrained '
            'hardware; headless mode avoids the GUI compositing/render cost.'
        ),
    )

    gz_args_gui = f'{world_path} -r'
    gz_args_headless = f'{world_path} -r -s --headless-rendering'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression([f'"{gz_args_headless}" if "', headless, f'" == "true" else "{gz_args_gui}"']),
        }.items(),
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
        }],
    )

    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start a single RViz2 instance showing both robots (map, TF, both global plans)',
    )
    rviz_config_path = os.path.join(pkg_amr_description_bringup, 'rviz', 'multi_robot_rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    tf_merge_relay_node = Node(
        package='amr_description_bringup',
        executable='tf_merge_relay.py',
        name='tf_merge_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    robot_groups = [
        TimerAction(
            period=idx * ROBOT_STAGGER_DELAY_S,
            actions=[make_robot_group(
                robot, urdf_path, nav2_params_path, pkg_nav2_bringup, use_sim_time)],
        )
        for idx, robot in enumerate(ROBOTS)
    ]

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_headless_arg,
        declare_use_rviz_arg,
        gazebo,
        clock_bridge_node,
        map_server_node,
        lifecycle_manager_map_node,
        rviz_node,
        tf_merge_relay_node,
        *robot_groups,
    ])
