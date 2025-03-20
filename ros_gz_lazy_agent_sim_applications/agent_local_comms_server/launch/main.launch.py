"""Launchfile for the cpp_epuck package."""

import launch
from launch import SomeSubstitutionsType
from launch.actions import (
    IncludeLaunchDescription as IncludeLaunch,
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
    'comms_manager_implementation': 'central_node_py',
    'agent_comms_implementation': 'udp_cpp',
    # 'agent_comms_implementation': 'udp_py',
    # 'agent_comms_implementation': 'gz_rf_py',
    'manager_server_host': '127.0.0.1',
    'manager_server_port': 50000,
    'manager_threshold_dist': 0.1,
    'manager_robot_tf_prefix': 'epuck2_robot_',
    'manager_robot_tf_suffix': '',
    'manager_robot_tf_frame': '/base_link',
    'agents': [
        {
            'robot_id': 0,
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10000,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50001,
            'robot_knowledge_host': '127.0.0.1',
            'robot_knowledge_exchange_port': 50002,
            'robot_xpos': 0.0,
            'robot_ypos': 0.0,
            'robot_theta': 0.0,
            'robot_teleop': False,
        },
        {
            'robot_id': 1,
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10000,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50001,
            'robot_knowledge_host': '127.0.0.1',
            'robot_knowledge_exchange_port': 50002,
            'robot_xpos': 0.0,
            'robot_ypos': 0.0,
            'robot_theta': 0.0,
            'robot_teleop': False,
        },
        {
            'robot_id': 2,
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10000,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50001,
            'robot_knowledge_host': '127.0.0.1',
            'robot_knowledge_exchange_port': 50002,
            'robot_xpos': 0.0,
            'robot_ypos': 0.0,
            'robot_theta': 0.0,
            'robot_teleop': False,
        },
        {
            'robot_id': 3,
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10000,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50001,
            'robot_knowledge_host': '127.0.0.1',
            'robot_knowledge_exchange_port': 50002,
            'robot_xpos': 0.0,
            'robot_ypos': 0.0,
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
            'rviz_config': [
                'config',
                'multi_epuck2_driver_rviz.rviz',
            ]
        },
        'gz_model_py': {
            'package': 'ros_gz_lazy_agent_sim_bringup',
            'launchfile': 'epuck2.launch.py',
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
            'launchfile': 'udp_agents.launch.py',
        },
        'gz_rf_py': {
            'package': 'agent_local_comms_server',
            'launchfile': 'gz_agents.launch.py',
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
    result = []

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

    result.append(_rviz)

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

    result.append(_static_tf_pub)

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
                'epuck2_id': agent['robot_id'],
                'epuck2_address': agent['robot_epuck_host'],
                'epuck2_port': agent['robot_epuck_port'],
                'epuck2_name':
                    f'{launch_configuration["manager_robot_tf_prefix"]}{i}',
                'cam_en': 'false',
                'floor_en': 'false',
                'xpos': agent['robot_xpos'],
                'ypos': agent['robot_ypos'],
                'theta': agent['robot_theta'],
                'is_single_robot': 'false',
                'sim_en': 'false',
            }.items(),
        )

        result.append(_include)

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
            'server_host': launch_configuration['manager_server_host'],
            'server_port': launch_configuration['manager_server_port'],
            'threshold_dist': launch_configuration['manager_threshold_dist'],
            'robot_tf_prefix': launch_configuration['manager_robot_tf_prefix'],
            'robot_tf_suffix': launch_configuration['manager_robot_tf_suffix'],
            'robot_tf_frame': launch_configuration['manager_robot_tf_frame'],
            **{
                f'remap_ids/{i}': agent['robot_id']
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
                'robot_id': agent['robot_id'],
                'manager_host': launch_configuration['manager_server_host'],
                'manager_port': launch_configuration['manager_server_port'],
                'robot_comms_host': agent['robot_comms_host'],
                'robot_comms_request_port': agent['robot_comms_request_port'],
                'robot_knowledge_host': agent['robot_knowledge_host'],
                'robot_knowledge_exchange_port':
                    agent['robot_knowledge_exchange_port'],
            }.items(),
        )

        result.append(_include)

    return result


def launch_teleop() -> list[launch.Action]:
    """Launch teleop nodes for robots that require it."""

    def get_namespace(i: int):
        return (
            launch_configuration['manager_robot_tf_prefix'],
            f'{i}',
            launch_configuration['manager_robot_tf_suffix'],
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
                        launch_configuration['manager_robot_tf_frame'],
                    ),
                    '"',
                ],
            ),
            shell=True,
        )

        result.append(_teleop)

    return result


def generate_launch_description():
    """Generate the main launch description."""
    ld = launch.LaunchDescription(
        include_epuck_implementation()
        + include_comms_manager_implementation()
        + include_agent_comms_implementations()
        + launch_teleop()
        + []
    )
    return ld


if __name__ == '__main__':
    generate_launch_description()
