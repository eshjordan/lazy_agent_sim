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
    # 'epuck_implementation': 'epuck_driver_cpp',
    'epuck_implementation': 'gz_model_py',
    'comms_manager_implementation': 'central_node_py',
    # 'agent_comms_implementation': 'udp_cpp',
    # 'agent_comms_implementation': 'udp_py',
    'agent_comms_implementation': 'gz_rf_py',
    'manager_server_host': '192.168.0.2',
    'manager_server_port': 50000,
    'manager_threshold_dist': 0.1,
    'manager_robot_tf_prefix': 'epuck2_robot_',
    'manager_robot_tf_suffix': '',
    'manager_robot_tf_frame': '/base_link',
    'agents': [
        {
            'robot_id': 0,
            'robot_epuck_host': '192.168.0.2',
            'robot_epuck_port': 10000,
            'robot_comms_host': '192.168.0.2',
            'robot_comms_request_port': 50002,
            'robot_knowledge_host': '192.168.0.2',
            'robot_knowledge_exchange_port': 50003,
            'robot_xpos': -0.1,
            'robot_ypos': -0.1,
            'robot_theta': 0.0,
            'robot_teleop': True,
        },
        {
            'robot_id': 1,
            'robot_epuck_host': '192.168.0.2',
            'robot_epuck_port': 10001,
            'robot_comms_host': '192.168.0.2',
            'robot_comms_request_port': 50004,
            'robot_knowledge_host': '192.168.0.2',
            'robot_knowledge_exchange_port': 50005,
            'robot_xpos': -0.1,
            'robot_ypos': 0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
        },
        {
            'robot_id': 2,
            'robot_epuck_host': '192.168.0.2',
            'robot_epuck_port': 10002,
            'robot_comms_host': '192.168.0.2',
            'robot_comms_request_port': 50006,
            'robot_knowledge_host': '192.168.0.2',
            'robot_knowledge_exchange_port': 50007,
            'robot_xpos': 0.1,
            'robot_ypos': -0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
        },
        {
            'robot_id': 3,
            'robot_epuck_host': '192.168.0.2',
            'robot_epuck_port': 10003,
            'robot_comms_host': '192.168.0.2',
            'robot_comms_request_port': 50008,
            'robot_knowledge_host': '192.168.0.2',
            'robot_knowledge_exchange_port': 50009,
            'robot_xpos': 0.1,
            'robot_ypos': 0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
        },
    ],
}


implementations = {
    'epuck_implementation': {
        'epuck_driver_cpp': {
            'package': 'epuck_driver_cpp',
            'launchfile': 'epuck2_controller.launch.py',
            'oneshot': False,
            'rviz_config': os.path.join(
                'config',
                'multi_epuck2_driver_rviz.rviz',
            ),
        },
        'gz_model_py': {
            'package': 'ros_gz_lazy_agent_sim_bringup',
            'launchfile': 'epuck2.launch.py',
            'oneshot': True,
            'rviz_config': os.path.join(
                'config',
                'epuck2.rviz',
            ),
        },
    },
    'comms_manager_implementation': {
        'central_node_py': {
            'package': 'agent_local_comms_server',
            'launchfile': 'agent_local_comms_server.launch.py',
        },
    },
    'agent_comms_implementation': {
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
    ] else ''


def include_epuck_implementation() -> list[launch.Action]:
    """Include the epuck implementation."""

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
                    get_implementation_value(
                        'epuck_implementation',
                        'rviz_config',
                    ),
                ]
            ),
        ],
    )

    _static_tf_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=[
            '--frame-id',
            'world',
            '--child-frame-id',
            'odom',
        ],
    )

    nodes = []

    if not get_implementation_value('epuck_implementation', 'oneshot'):
        # breakpoint()  # not oneshot
        for i, agent in enumerate(launch_configuration['agents']):
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
                launch_arguments={
                    'namespace':
                        f'{launch_configuration["manager_robot_tf_prefix"]}{i}',
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
                }.items(),
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
        )

        nodes.append(_include)

    retval = [
        _rviz,
        TimerAction(
            period=2.0,
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        _static_tf_pub,
                        TimerAction(
                            period=2.0,
                            actions=nodes,
                        )
                    ]
                )
            ]
        )
    ]

    return retval


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
                for i, agent in enumerate(launch_configuration['agents'])
            },
        }.items(),
    )

    result.append(_include)

    return result


def include_agent_comms_implementations() -> list[launch.Action]:
    """Include the agent comms implementations."""
    result = []

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
                'manager_host':
                    f"{launch_configuration['manager_server_host']}",
                'manager_port':
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


def generate_launch_description():
    """Generate the main launch description."""
    actions = include_epuck_implementation() + \
        include_comms_manager_implementation() + \
        include_agent_comms_implementations() + \
        launch_teleop()

    ld = launch.LaunchDescription(
        actions
    )

    # breakpoint()

    return ld


# if __name__ == '__main__':
#     generate_launch_description()
