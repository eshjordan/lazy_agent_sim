"""Launchfile for the cpp_epuck package."""

import os
import launch
from launch import SomeSubstitutionsType
from launch.actions import (
    IncludeLaunchDescription as IncludeLaunch,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource as PythonLaunch,
)
from launch.substitutions import (
    PathJoinSubstitution as PathJoin,
)

import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


launch_configuration = {
    'epuck_implementation': 'epuck_driver_cpp',
    # 'epuck_implementation': 'gz_model_py',
    # 'epuck_implementation': 'gz_model_headless_py',
    'comms_manager_implementation': 'central_node_py',
    'localisation_implementation': 'gz_localisation',
    'waypoint_controller_implementation': 'waypoint_controller_py',
    'agent_comms_implementation': 'epuck_firmware',
    # 'agent_comms_implementation': 'udp_cpp',
    # 'agent_comms_implementation': 'udp_py',
    # 'agent_comms_implementation': 'gz_rf_py',
    'manager_server_host': '192.168.11.5',
    'manager_server_port': 50000,
    'manager_threshold_dist': 0.3,
    'manager_robot_tf_prefix': 'epuck2_robot_',
    'manager_robot_tf_suffix': '',
    'manager_robot_tf_frame': '/base_link',
    'agents': [
        {
            'robot_id': 5785,
            'robot_epuck_host': '192.168.11.11',
            'robot_epuck_port': 1000,
            'robot_comms_host': '192.168.11.11',
            'robot_comms_request_port': 1001,
            'robot_knowledge_host': '192.168.11.11',
            'robot_knowledge_exchange_port': 1002,
            'robot_xpos': -0.1,
            'robot_ypos': -0.1,
            'robot_theta': 0.0,
            'robot_teleop': True,
            'robot_angular_offset': -2.360,
        },
        {
            'robot_id': 5653,
            'robot_epuck_host': '192.168.11.12',
            'robot_epuck_port': 1000,
            'robot_comms_host': '192.168.11.12',
            'robot_comms_request_port': 1001,
            'robot_knowledge_host': '192.168.11.12',
            'robot_knowledge_exchange_port': 1002,
            'robot_xpos': -0.1,
            'robot_ypos': 0.1,
            'robot_theta': 0.0,
            'robot_teleop': True,
            'robot_angular_offset': -2.360,
        },
        # {
        #     'robot_id': 5731,
        #     'robot_epuck_host': '192.168.0.21',
        #     'robot_epuck_port': 1000,
        #     'robot_comms_host': '192.168.0.21',
        #     'robot_comms_request_port': 1001,
        #     'robot_knowledge_host': '192.168.0.21',
        #     'robot_knowledge_exchange_port': 1002,
        #     'robot_xpos': -0.1,
        #     'robot_ypos': -0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': True,
        #     'robot_angular_offset': -2.360,
        # },
        # {
        #     'robot_id': 5831,
        #     'robot_epuck_host': '192.168.0.22',
        #     'robot_epuck_port': 1000,
        #     'robot_comms_host': '192.168.0.22',
        #     'robot_comms_request_port': 1001,
        #     'robot_knowledge_host': '192.168.0.22',
        #     'robot_knowledge_exchange_port': 1002,
        #     'robot_xpos': -0.1,
        #     'robot_ypos': 0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': False,
        #     'robot_angular_offset': -2.360,
        # },
        # {
        #     'robot_id': 0,
        #     'robot_epuck_host': '192.168.0.2',
        #     'robot_epuck_port': 10000,
        #     'robot_comms_host': '192.168.0.2',
        #     'robot_comms_request_port': 50002,
        #     'robot_knowledge_host': 'aa:bb:cc:dd:ee:00',
        #     'robot_knowledge_exchange_port': 50003,
        #     'robot_xpos': -0.1,
        #     'robot_ypos': -0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': True,
        #     'robot_angular_offset': 0.0,
        # },
        # {
        #     'robot_id': 1,
        #     'robot_epuck_host': '192.168.0.2',
        #     'robot_epuck_port': 10001,
        #     'robot_comms_host': '192.168.0.2',
        #     'robot_comms_request_port': 50004,
        #     'robot_knowledge_host': 'aa:bb:cc:dd:ee:01',
        #     'robot_knowledge_exchange_port': 50005,
        #     'robot_xpos': -0.1,
        #     'robot_ypos': 0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': False,
        #     'robot_angular_offset': 0.0,
        # },
        # {
        #     'robot_id': 2,
        #     'robot_epuck_host': '192.168.0.2',
        #     'robot_epuck_port': 10002,
        #     'robot_comms_host': '192.168.0.2',
        #     'robot_comms_request_port': 50006,
        #     'robot_knowledge_host': 'aa:bb:cc:dd:ee:02',
        #     'robot_knowledge_exchange_port': 50007,
        #     'robot_xpos': 0.1,
        #     'robot_ypos': -0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': False,
        #     'robot_angular_offset': 0.0,
        # },
        # {
        #     'robot_id': 3,
        #     'robot_epuck_host': '192.168.0.2',
        #     'robot_epuck_port': 10003,
        #     'robot_comms_host': '192.168.0.2',
        #     'robot_comms_request_port': 50008,
        #     'robot_knowledge_host': 'aa:bb:cc:dd:ee:03',
        #     'robot_knowledge_exchange_port': 50009,
        #     'robot_xpos': 0.1,
        #     'robot_ypos': 0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': False,
        #     'robot_angular_offset': 0.0,
        # },
    ],
    'static_transforms': [
        {
            'frame-id': 'earth',
            'child-frame-id': 'epuck2_robot_0',
            'x': '-0.1',
            'y': '-0.1',
        },
        {
            'frame-id': 'epuck2_robot_0/map',
            'child-frame-id': 'epuck2_robot_0/odom',
        },
        {
            'frame-id': 'earth',
            'child-frame-id': 'epuck2_robot_1',
            'x': '-0.1',
            'y': '0.1',
        },
        {
            'frame-id': 'epuck2_robot_1/map',
            'child-frame-id': 'epuck2_robot_1/odom',
        },
        {
            'frame-id': 'earth',
            'child-frame-id': 'epuck2_robot_2',
            'x': '0.1',
            'y': '-0.1',
        },
        {
            'frame-id': 'epuck2_robot_2/map',
            'child-frame-id': 'epuck2_robot_2/odom',
        },
        {
            'frame-id': 'earth',
            'child-frame-id': 'epuck2_robot_3',
            'x': '0.1',
            'y': '0.1',
        },
        {
            'frame-id': 'epuck2_robot_3/map',
            'child-frame-id': 'epuck2_robot_3/odom',
        },
    ],
}


implementations = {
    'epuck_implementation': {
        'epuck_driver_cpp': {
            'package': 'epuck_driver_cpp',
            'launchfile': 'epuck2_controller.launch.py',
            'oneshot': False,
            # 'rviz_config': os.path.join(
            #     'config',
            #     'multi_epuck2_driver_rviz.rviz',
            # ),
        },
        'gz_model_py': {
            'package': 'ros_gz_lazy_agent_sim_bringup',
            'launchfile': 'epuck2.launch.py',
            'oneshot': True,
            'rviz_config': os.path.join(
                'config',
                'epuck2.rviz',
            ),
            'extra_args': {
                'gui': 'true',
            },
        },
        'gz_model_headless_py': {
            'package': 'ros_gz_lazy_agent_sim_bringup',
            'launchfile': 'epuck2.launch.py',
            'oneshot': True,
            'rviz_config': os.path.join(
                'config',
                'epuck2.rviz',
            ),
            'extra_args': {
                'gui': 'false',
            },
        },
    },
    'comms_manager_implementation': {
        'central_node_py': {
            'package': 'agent_local_comms_server',
            'launchfile': 'agent_local_comms_server.launch.py',
        },
    },
    'agent_comms_implementation': {
        'epuck_firmware': {
        },
        'udp_cpp': {
            'package': 'cpp_epuck',
            'launchfile': 'main.launch.py',
        },
        'udp_py': {
            'package': 'agent_local_comms_server',
            'launchfile': 'udp_agent.launch.py',
        },
        'gz_rf_py': {
            'package': 'agent_local_comms_server',
            'launchfile': 'gz_agent.launch.py',
        },
    },
    'localisation_implementation': {
        'gz_localisation': {
            'package': 'localisation_bringup',
            'launchfile': 'main.launch.py',
        }
    },
    'waypoint_controller_implementation': {
        'waypoint_controller_py': {
            'package': 'waypoint_controller',
            'launchfile': 'waypoint_controller_test.launch.py',
            'oneshot': False,
            'extra_args': {
                'min_linear_vel': '0.05',
                'max_linear_vel': '0.1',
                'min_angular_vel': '0.0',
                'max_angular_vel': '3.14',
                'slow_distance': '0.3',
                'slow_angle': '0.3',
                'threshold_distance': '0.05',
                'threshold_angle': '0.05',
            },
        },
    },
}


def get_implementation_value(
    implementation: str, value: str
) -> SomeSubstitutionsType:
    """Get an implementation-dependent value from the launch configuration.

    Allows the user to select a single top-level implementation with common
    settings without needing to update values in multiple places.

    Args:
        implementation (str): Which implementation type to use.
        value (str): The value to be substituted.

    Returns:
        SomeSubstitutionsType: The substitution.
    """
    return implementations[implementation][
        launch_configuration[implementation]
    ][value] if value in implementations[implementation][
        launch_configuration[implementation]
    ] else None


def include_epuck_implementation() -> list[launch.Action]:
    """Include the epuck implementation."""
    result = []

    rviz_config = get_implementation_value(
        'epuck_implementation',
        'rviz_config',
    )

    extra_args = get_implementation_value(
        'epuck_implementation',
        'extra_args',
    )

    if rviz_config:
        _rviz = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoin(
                    [
                        FindPackageShare(
                            get_implementation_value(
                                'epuck_implementation',
                                'package',
                            ),
                        ),
                        rviz_config,
                    ]
                ),
            ],
            ros_arguments=[
                '--disable-stdout-logs'
            ]
        )

        result.append(_rviz)

    nodes = []

    if not get_implementation_value('epuck_implementation', 'oneshot'):
        # breakpoint()  # not oneshot
        for i, agent in enumerate(launch_configuration['agents']):

            launch_arguments = {
                'namespace':
                    f'/{launch_configuration["manager_robot_tf_prefix"]}{i}',
                'epuck2_id': f"{agent['robot_id']}",
                'epuck2_address': f"{agent['robot_epuck_host']}",
                'epuck2_port': f"{agent['robot_epuck_port']}",
                'epuck2_name':
                    f'{launch_configuration["manager_robot_tf_prefix"]}{i}',
                'cam_en': 'false',
                'floor_en': 'false',
                'xpos': f"{agent['robot_xpos']}",
                'ypos': f"{agent['robot_ypos']}",
                'theta': f"{agent['robot_theta']}",
                'is_single_robot': 'false',
                'sim_en': 'false',
            }

            launch_arguments.update(extra_args if extra_args else {})

            _include = IncludeLaunch(
                PythonLaunch(
                    PathJoin(
                        [
                            FindPackageShare(
                                get_implementation_value(
                                    'epuck_implementation',
                                    'package'
                                ),
                            ),
                            'launch',
                            get_implementation_value(
                                'epuck_implementation',
                                'launchfile',
                            ),
                        ]
                    )
                ),
                launch_arguments=launch_arguments.items(),
            )

            nodes.append(_include)
    else:
        # breakpoint()  # oneshot
        _include = IncludeLaunch(
            PythonLaunch(
                PathJoin(
                    [
                        FindPackageShare(
                            get_implementation_value(
                                'epuck_implementation',
                                'package',
                            ),
                        ),
                        'launch',
                        get_implementation_value(
                            'epuck_implementation',
                            'launchfile',
                        ),
                    ]
                )
            ),
            launch_arguments=extra_args.items(),
        )

        nodes.append(_include)

    result.append(TimerAction(period=2.0, actions=nodes))

    return result


def include_comms_manager_implementation() -> list[launch.Action]:
    """Include the comms manager implementation."""
    result = []

    _include = IncludeLaunch(
        PythonLaunch(
            PathJoin(
                [
                    FindPackageShare(
                        get_implementation_value(
                            'comms_manager_implementation',
                            'package',
                        ),
                    ),
                    'launch',
                    get_implementation_value(
                        'comms_manager_implementation',
                        'launchfile',
                    ),
                ]
            )
        ),
        launch_arguments={
            'server_host':
                f"{launch_configuration['manager_server_host']}",
            'server_port':
                f"{launch_configuration['manager_server_port']}",
            'threshold_dist':
                f"{launch_configuration['manager_threshold_dist']}",
            'robot_tf_prefix':
                f"{launch_configuration['manager_robot_tf_prefix']}",
            'robot_tf_suffix':
                f"{launch_configuration['manager_robot_tf_suffix']}",
            'robot_tf_frame':
                f"{launch_configuration['manager_robot_tf_frame']}",
            **{
                f'remap_ids/{i}': f"{agent['robot_id']}"
                for i, agent in enumerate(launch_configuration['agents'])},
        }.items(),
    )

    result.append(_include)

    return result


def include_agent_comms_implementations() -> list[launch.Action]:
    """Include the agent comms implementations."""
    result = []

    launchfile = get_implementation_value(
        'agent_comms_implementation',
        'launchfile',
    )

    if not launchfile:
        return result

    for i, agent in enumerate(launch_configuration['agents']):
        _include = IncludeLaunch(
            PythonLaunch(
                PathJoin(
                    [
                        FindPackageShare(
                            get_implementation_value(
                                'agent_comms_implementation',
                                'package',
                            ),
                        ),
                        'launch',
                        get_implementation_value(
                            'agent_comms_implementation',
                            'launchfile',
                        ),
                    ]
                )
            ),
            launch_arguments={
                'robot_id':
                    f"{agent['robot_id']}",
                'manager_server_host':
                    f"{launch_configuration['manager_server_host']}",
                'manager_server_port':
                    f"{launch_configuration['manager_server_port']}",
                'robot_comms_host':
                    f"{agent['robot_comms_host']}",
                'robot_comms_request_port':
                    f"{agent['robot_comms_request_port']}",
                'robot_knowledge_host':
                    f"{agent['robot_knowledge_host']}",
                'robot_knowledge_exchange_port':
                    f"{agent['robot_knowledge_exchange_port']}",
            }.items(),
        )

        result.append(_include)

    return result


def include_localisation_implementation() -> list[launch.Action]:
    """Include the localisation implementation."""
    result = []
    return result

    launchfile = get_implementation_value(
        'localisation_implementation',
        'launchfile',
    )

    if not launchfile:
        return result

    for i, agent in enumerate(launch_configuration['agents']):
        _include = IncludeLaunch(
            PythonLaunch(
                PathJoin(
                    [
                        FindPackageShare(
                            get_implementation_value(
                                'localisation_implementation',
                                'package',
                            ),
                        ),
                        'launch',
                        get_implementation_value(
                            'localisation_implementation',
                            'launchfile',
                        ),
                    ]
                )
            ),
            launch_arguments={
                'id': f'{i}',
                'robot_tf_prefix':
                    f"{launch_configuration['manager_robot_tf_prefix']}",
                'robot_tf_suffix':
                    f"{launch_configuration['manager_robot_tf_suffix']}",
            }.items(),
        )

        result.append(_include)

    return result


def include_waypoint_controller_implementation() -> list[launch.Action]:
    """Include the waypoint controller implementation."""
    result = []

    launchfile = get_implementation_value(
        'waypoint_controller_implementation',
        'launchfile',
    )

    if not launchfile:
        return result

    extra_args = get_implementation_value(
        'epuck_implementation',
        'extra_args',
    )

    if not get_implementation_value('waypoint_controller_implementation', 'oneshot'):
        for i, agent in enumerate(launch_configuration['agents']):
            launch_arguments = {
                'namespace':
                    f'/{launch_configuration["manager_robot_tf_prefix"]}{i}',
                'robot_id':
                    f"{agent['robot_id']}",
                'robot_tf_prefix':
                    f"{launch_configuration['manager_robot_tf_prefix']}",
                'robot_tf_suffix':
                    f"{launch_configuration['manager_robot_tf_suffix']}",
                'robot_tf_frame':
                    f"{launch_configuration['manager_robot_tf_frame']}",
                'min_linear_vel': '0.05',
                'max_linear_vel': '0.1',
                'min_angular_vel': '0.0',
                'max_angular_vel': '3.14',
                'slow_distance': '0.3',
                'slow_angle': '0.3',
                'threshold_distance': '0.05',
                'threshold_angle': '0.05',
                'angular_offset': f"{agent['robot_angular_offset']}",
            }

            # launch_arguments.update(extra_args if extra_args else {})

            _include = IncludeLaunch(
                PythonLaunch(
                    PathJoin(
                        [
                            FindPackageShare(
                                get_implementation_value(
                                    'waypoint_controller_implementation',
                                    'package',
                                ),
                            ),
                            'launch',
                            get_implementation_value(
                                'waypoint_controller_implementation',
                                'launchfile',
                            ),
                        ]
                    )
                ),
                launch_arguments=launch_arguments.items(),
            )

            result.append(_include)
    else:
        launch_arguments = {
            'namespace':
                f'/{launch_configuration["manager_robot_tf_prefix"]}',
            'robot_tf_prefix':
                f"{launch_configuration['manager_robot_tf_prefix']}",
            'robot_tf_suffix':
                f"{launch_configuration['manager_robot_tf_suffix']}",
            'robot_tf_frame':
                f"{launch_configuration['manager_robot_tf_frame']}",
        }

        launch_arguments.update(extra_args if extra_args else {})

        _include = IncludeLaunch(
            PythonLaunch(
                PathJoin(
                    [
                        FindPackageShare(
                            get_implementation_value(
                                'waypoint_controller_implementation',
                                'package',
                            ),
                        ),
                        'launch',
                        get_implementation_value(
                            'waypoint_controller_implementation',
                            'launchfile',
                        ),
                    ]
                )
            ),
            launch_arguments=launch_arguments.items(),
        )

        result.append(_include)

    return result


def launch_teleop() -> list[launch.Action]:
    """Launch teleop nodes for robots that require it."""

    def get_namespace(i: int):
        return (
            f"{launch_configuration['manager_robot_tf_prefix']}",
            f'{i}',
            f"{launch_configuration['manager_robot_tf_suffix']}",
        )

    result = []

    for i, agent in enumerate(launch_configuration['agents']):
        if not agent['robot_teleop']:
            continue

        _teleop = launch.actions.ExecuteLocal(
            process_description=launch.descriptions.Executable(
                cmd=[
                    'xterm',
                    '-bg black -fg white -fa "Monospace" -fs 13 -title "',
                    (*get_namespace(i), '/mobile_base/cmd_vel"'),
                    '-e "',
                    'ros2',
                    'run',
                    'teleop_twist_keyboard',
                    'teleop_twist_keyboard',
                    '--ros-args',
                    '-r',
                    ('__ns:=/', *get_namespace(i), '/mobile_base'),
                    '-r',
                    'stamped:=true',
                    '-r',
                    (
                        'frame_id:=',
                        *get_namespace(i),
                        f"{launch_configuration['manager_robot_tf_frame']}",
                    ),
                    '"',
                ],
            ),
            shell=True,
        )

        result.append(_teleop)

    retval = []

    for r in result:
        retval.append(TimerAction(period=2.0, actions=[r]))

    return retval


def launch_static_transforms() -> list[launch.Action]:
    """Launch static transforms for the robots."""

    result = []

    for i, transform in enumerate(launch_configuration['static_transforms']):
        arguments = []
        for k, v in transform.items():
            arguments.append(f'--{k}')
            arguments.append(f'{v}')

        _static_transform = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_{i}',
            output='screen',
            arguments=arguments,
        )

        result.append(_static_transform)

    return result


def generate_launch_description():
    """Generate the main launch description."""
    actions = include_epuck_implementation() + \
        include_comms_manager_implementation() + \
        include_agent_comms_implementations() + \
        include_waypoint_controller_implementation() + \
        include_localisation_implementation() + \
        launch_static_transforms() + \
        launch_teleop()

    ld = launch.LaunchDescription(
        actions + [
            launch_ros.actions.Node(
                executable='pose_tf',
                package='agent_local_comms_server',
                parameters=[
                    {
                        'topic_name': '/vrpn_mocap/BW_epuck0/pose',
                        'frame_id': 'earth',
                        'child_frame_id': 'epuck2_robot_0/base_link',
                    }
                ]
            ),
            launch_ros.actions.Node(
                executable='pose_tf',
                package='agent_local_comms_server',
                parameters=[
                    {
                        'topic_name': '/vrpn_mocap/BW_epuck1/pose',
                        'frame_id': 'earth',
                        'child_frame_id': 'epuck2_robot_1/base_link',
                    }
                ]
            ),
            # ros2 launch vrpn_mocap client.launch.yaml server:=192.168.11.3 port:=3883
            # sudo pkill -f static_transform_publisher ; sudo pkill -f controller ; sudo pkill -f agent_local_comms_server
        ]
    )

    # breakpoint()

    return ld


# if __name__ == '__main__':
#     generate_launch_description()
