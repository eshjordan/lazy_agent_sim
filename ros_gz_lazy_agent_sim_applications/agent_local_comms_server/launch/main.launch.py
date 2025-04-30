"""Launchfile for the cpp_epuck package."""

import os
from typing import Dict, Iterable, Optional, Tuple
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
import launch_ros.parameters_type
from launch_ros.substitutions import FindPackageShare

use_gazebo = True

epuck_config = {
    'epuck_implementation': 'epuck_driver_cpp',
    'comms_manager_implementation': 'central_node_py',
    'localisation_implementation': 'vicon_localisation',
    'waypoint_controller_implementation': 'waypoint_controller_py',
    'agent_comms_implementation': 'epuck_firmware',
    'mocap_implementation': 'vrpn_mocap',
    'manager_server_host': '192.168.11.5',
    'manager_server_port': 50000,
    'manager_threshold_dist': 0.3,
    'manager_robot_tf_prefix': 'epuck2_robot_',
    'manager_robot_tf_suffix': '',
    'manager_robot_tf_frame': '/base_link',
    'agents': [
        {
            'robot_id': 5785,
            'robot_name': 'epuck2_robot_{agent["robot_id"]}',
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
            'robot_vicon_name': 'BW_epuck0',
        },
        {
            'robot_id': 5653,
            'robot_name': 'epuck2_robot_{agent["robot_id"]}',
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
            'robot_vicon_name': 'BW_epuck1',
        },
        # {
        #     'robot_id': 5731,
        #     'robot_name': 'epuck2_robot_{agent["robot_id"]}',
        #     'robot_epuck_host': '192.168.11.13',
        #     'robot_epuck_port': 1000,
        #     'robot_comms_host': '192.168.11.13',
        #     'robot_comms_request_port': 1001,
        #     'robot_knowledge_host': '192.168.11.13',
        #     'robot_knowledge_exchange_port': 1002,
        #     'robot_xpos': -0.1,
        #     'robot_ypos': -0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': True,
        #     'robot_angular_offset': -2.360,
        #     'robot_vicon_name': 'BW_epuck2',
        # },
        # {
        #     'robot_id': 5831,
        #     'robot_name': 'epuck2_robot_{agent["robot_id"]}',
        #     'robot_epuck_host': '192.168.11.14',
        #     'robot_epuck_port': 1000,
        #     'robot_comms_host': '192.168.11.14',
        #     'robot_comms_request_port': 1001,
        #     'robot_knowledge_host': '192.168.11.14',
        #     'robot_knowledge_exchange_port': 1002,
        #     'robot_xpos': -0.1,
        #     'robot_ypos': 0.1,
        #     'robot_theta': 0.0,
        #     'robot_teleop': False,
        #     'robot_angular_offset': -2.360,
        #     'robot_vicon_name': 'BW_epuck3',
        # },
    ],
}

gazebo_config = {
    'epuck_implementation': 'gz_model_headless_py',
    'comms_manager_implementation': 'central_node_py',
    'localisation_implementation': 'gz_localisation',
    'waypoint_controller_implementation': 'waypoint_controller_py',
    'agent_comms_implementation': 'gz_rf_py',
    'mocap_implementation': 'none',
    'manager_server_host': '127.0.0.1',
    'manager_server_port': 50000,
    'manager_threshold_dist': 0.3,
    'manager_robot_tf_prefix': 'epuck2_robot_',
    'manager_robot_tf_suffix': '',
    'manager_robot_tf_frame': '/base_link',
    'agents': [
        {
            'robot_id': 0,
            'robot_name': 'epuck2_robot_{agent["robot_id"]}',
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10000,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50002,
            'robot_knowledge_host': 'aa:bb:cc:dd:ee:00',
            'robot_knowledge_exchange_port': 50003,
            'robot_xpos': -0.1,
            'robot_ypos': -0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
            'robot_angular_offset': 0.0,
            'robot_vicon_name': 'BW_epuck0',
        },
        {
            'robot_id': 1,
            'robot_name': 'epuck2_robot_{agent["robot_id"]}',
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10001,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50004,
            'robot_knowledge_host': 'aa:bb:cc:dd:ee:01',
            'robot_knowledge_exchange_port': 50005,
            'robot_xpos': -0.1,
            'robot_ypos': 0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
            'robot_angular_offset': 0.0,
            'robot_vicon_name': 'BW_epuck1',
        },
        {
            'robot_id': 2,
            'robot_name': 'epuck2_robot_{agent["robot_id"]}',
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10002,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50006,
            'robot_knowledge_host': 'aa:bb:cc:dd:ee:02',
            'robot_knowledge_exchange_port': 50007,
            'robot_xpos': 0.1,
            'robot_ypos': -0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
            'robot_angular_offset': 0.0,
            'robot_vicon_name': 'BW_epuck2',
        },
        {
            'robot_id': 3,
            'robot_name': 'epuck2_robot_{agent["robot_id"]}',
            'robot_epuck_host': '127.0.0.1',
            'robot_epuck_port': 10003,
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': 50008,
            'robot_knowledge_host': 'aa:bb:cc:dd:ee:03',
            'robot_knowledge_exchange_port': 50009,
            'robot_xpos': 0.1,
            'robot_ypos': 0.1,
            'robot_theta': 0.0,
            'robot_teleop': False,
            'robot_angular_offset': 0.0,
            'robot_vicon_name': 'BW_epuck3',
        },
    ],
}

common_config = {
    'static_transforms': [
    ],
    'static_agent_transforms': [
        {
            'frame-id': 'earth',
            'child-frame-id': '{}',
            'x': '{}',
            'y': '{}',
        },
        {
            'frame-id': '{}',
            'child-frame-id': '{}/map',
        },
        # {
        #     'frame-id': '{}/map',
        #     'child-frame-id': '{}/odom',
        # },
    ]
}

launch_configuration = common_config
if use_gazebo:
    launch_configuration.update(gazebo_config)
else:
    launch_configuration.update(epuck_config)


implementations = {
    'epuck_implementation': {
        'none': {},
        'epuck_driver_cpp': {
            'package': 'epuck_driver_cpp',
            'launchfile': 'epuck2_controller.launch.py',
            'oneshot': False,
            'rviz_config': os.path.join(
                'config',
                'multi_epuck2_driver_rviz.rviz',
            ),
            'extra_args': {
                'rviz': 'false',
            },
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
                'gz_version': '8',
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
        'none': {},
        'central_node_py': {
            'package': 'agent_local_comms_server',
            'launchfile': 'agent_local_comms_server.launch.py',
        },
    },
    'agent_comms_implementation': {
        'none': {},
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
        'none': {},
        'gz_localisation': {
            'package': 'agent_local_comms_server',
            'launchfile': 'pose_tf.launch.py',
            'oneshot': False,
            'extra_args': {
                'source_topic_name': '/{}/pose',
                'source_frame_id': 'earth',
                'source_child_frame_id': '{}/base_link',
                'tf_frame_id': '{}/map',
                'tf_child_frame_id': '{}/odom',
            },
        },
        'vicon_localisation': {
            'package': 'agent_local_comms_server',
            'launchfile': 'pose_tf.launch.py',
            'oneshot': False,
            'extra_args': {
                'source_topic_name': '/vrpn_mocap/{}/pose',
                'source_frame_id': 'earth',
                'source_child_frame_id': '{}/base_link',
                'tf_frame_id': '{}/map',
                'tf_child_frame_id': '{}/odom',
            },
        },
        'odom_localisation': {
        }
    },
    'waypoint_controller_implementation': {
        'none': {},
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
    'mocap_implementation': {
        'none': {},
        'vrpn_mocap': {
            'launch_type': 'node',  # 'python_launchfile', 'yaml_launchfile', 'xml_launchfile'
            'package': 'vrpn_mocap',
            'launch': 'client_node',
            'oneshot': True,
            'extra_args': {
                'launch_arguments': {
                    'source_topic_name': '/vrpn_client_node/{}/pose',
                    'source_frame_id': 'earth',
                    'source_child_frame_id': '{}/base_link',
                    'tf_frame_id': '{}/map',
                    'tf_child_frame_id': '{}/odom',
                },
                'executable': '',
                'package': '',
                'name': '',
                'namespace': '',
                'exec_name': '',
                'parameters': [],
                'remappings': [],
                'ros_arguments': [],
                'arguments': [],

                'env': '',
                'additional_env': '',
                'shell': '',
                'sigterm_timeout': '',
                'sigkill_timeout': '',
                'emulate_tty': '',
                'prefix': '',
                'output': '',
                'output_format': '',
                'log_cmd': '',
                'cached_output': '',
                'on_exit': '',
                'respawn': '',
                'respawn_delay': '',
                'respawn_max_retries': '',
                'condition': '',
            },
        },
    }
}


def launch_implementation(context, impl_category_definitions, config, impl_category) -> list[launch.Action]:
    """Launch the implementation for the selected robot."""

    def get_implementation_value(value: str) -> SomeSubstitutionsType:
        if impl_category not in impl_category_definitions:
            return None
        available_impls = impl_category_definitions[impl_category]
        if impl_category not in config:
            return None
        selected_impl = config[impl_category]
        if selected_impl not in available_impls:
            return None
        if value not in available_impls[config[impl_category]]:
            return None
        return impl_category_definitions[impl_category][config[impl_category]][value]

    def get_extra_arg(value: str) -> SomeSubstitutionsType:
        extra_args = get_implementation_value('extra_args')
        if extra_args is None:
            return None
        if value not in extra_args:
            return None
        return extra_args[value]

    def substitute_dict(d: Optional[Dict[str, str]], agent=None):
        return {f"{k}": f"{v}" for k, v in d.items()} if d else None

    def substitute_iterable(i: Optional[Iterable[str]], agent=None):
        return [f"{x}" for x in i] if i else None

    def substitute_string(s: Optional[str], agent=None):
        return f"{s}" if s else None

    def substitute_remap(r: Optional[Iterable[Tuple[str, str]]], agent=None):
        return [(f"{x[0]}", f"{x[1]}") for x in r] if r else None

    actions = []

    if get_implementation_value('oneshot'):

        if get_implementation_value('launch_type') == 'node':
            _action = launch_ros.actions.Node(
                executable=substitute_string(get_extra_arg('executable') if get_extra_arg(
                    'executable') else get_implementation_value('launch')),
                package=substitute_string(get_extra_arg('package') if get_extra_arg(
                    'package') else get_implementation_value('package')),
                name=substitute_string(get_extra_arg('name')),
                namespace=substitute_string(get_extra_arg('namespace')),
                exec_name=substitute_string(get_extra_arg('exec_name')),
                parameters=substitute_dict(get_extra_arg('parameters')),
                remappings=substitute_remap(get_extra_arg('remappings')),
                ros_arguments=substitute_iterable(
                    get_extra_arg('ros_arguments')),
                arguments=substitute_iterable(get_extra_arg('arguments')),
                env=substitute_dict(get_extra_arg('env')),
                additional_env=substitute_dict(
                    get_extra_arg('additional_env')),
                shell=substitute_string(get_extra_arg('shell')),
                sigterm_timeout=substitute_string(
                    get_extra_arg('sigterm_timeout')),
                sigkill_timeout=substitute_string(
                    get_extra_arg('sigkill_timeout')),
                emulate_tty=substitute_string(get_extra_arg('emulate_tty')),
                prefix=substitute_string(get_extra_arg('prefix')),
                output=substitute_string(get_extra_arg('output')),
                output_format=substitute_string(
                    get_extra_arg('output_format')),
                log_cmd=substitute_string(get_extra_arg('log_cmd')),
                cached_output=substitute_string(
                    get_extra_arg('cached_output')),
                # on_exit is a bit tricky to implement
                on_exit=substitute_string(get_extra_arg('on_exit')),
                respawn=substitute_string(get_extra_arg('respawn')),
                respawn_delay=substitute_string(
                    get_extra_arg('respawn_delay')),
                respawn_max_retries=substitute_string(get_extra_arg(
                    'respawn_max_retries')),
                condition=substitute_string(get_extra_arg('condition')),
            )

            actions.append(_action)

        elif get_implementation_value('launch_type') == 'python_launchfile':
            _action = IncludeLaunch(
                PythonLaunch(
                    PathJoin(
                        [
                            FindPackageShare(
                                get_implementation_value('package'),
                            ),
                            'launch',
                            get_implementation_value('launch'),
                        ]
                    )
                ),
                launch_arguments=get_implementation_value(
                    'launch_arguments').items(),
            )
            actions.append(_action)

    return actions


def get_implementation_value(
    impl_category: str, value: str
) -> SomeSubstitutionsType:
    """Get an implementation-dependent value from the launch configuration.

    Allows the user to select a single top-level implementation with common
    settings without needing to update values in multiple places.

    Args:
        impl_category (str): Which implementation type to use.
        value (str): The value to be substituted.

    Returns:
        SomeSubstitutionsType: The substitution.
    """
    if impl_category not in implementations:
        return None
    available_impls = implementations[impl_category]
    if impl_category not in launch_configuration:
        return None
    selected_impl = launch_configuration[impl_category]
    if selected_impl not in available_impls:
        return None
    if value not in available_impls[launch_configuration[impl_category]]:
        return None
    return implementations[impl_category][launch_configuration[impl_category]][value]


def include_epuck_implementation(context) -> list[launch.Action]:
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
        for agent in launch_configuration['agents']:
            robot_id = agent['robot_id']

            launch_arguments = {
                'namespace':
                    f'/{launch_configuration["manager_robot_tf_prefix"]}{robot_id}',
                'epuck2_id': f"{robot_id}",
                'epuck2_address': f"{agent['robot_epuck_host']}",
                'epuck2_port': f"{agent['robot_epuck_port']}",
                'epuck2_name':
                    f'{launch_configuration["manager_robot_tf_prefix"]}{robot_id}',
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


def include_comms_manager_implementation(context) -> list[launch.Action]:
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
            'robot_ids':
                f"{','.join([f"{agent['robot_id']}" for agent in launch_configuration['agents']])}",
        }.items(),
    )

    result.append(_include)

    return result


def include_agent_comms_implementations(context) -> list[launch.Action]:
    """Include the agent comms implementations."""
    result = []

    launchfile = get_implementation_value(
        'agent_comms_implementation',
        'launchfile',
    )

    if not launchfile:
        return result

    for agent in launch_configuration['agents']:
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


def include_localisation_implementation(context) -> list[launch.Action]:
    """Include the localisation implementation."""
    result = []

    launchfile = get_implementation_value(
        'localisation_implementation',
        'launchfile',
    )

    if not launchfile:
        return result

    extra_args = get_implementation_value(
        'localisation_implementation',
        'extra_args',
    )

    def get_namespace(i: int):
        return (
            f"{launch_configuration['manager_robot_tf_prefix']}"
            + f'{i}'
            + f"{launch_configuration['manager_robot_tf_suffix']}"
        )

    if not get_implementation_value('localisation_implementation', 'oneshot'):
        for i, agent in enumerate(launch_configuration['agents']):
            robot_id = agent['robot_id']

            launch_arguments = {}

            launch_arguments.update(extra_args if extra_args else {})
            if launch_configuration['localisation_implementation'] == 'vicon_localisation' or launch_configuration['localisation_implementation'] == 'gz_localisation':
                if 'source_topic_name' in launch_arguments:
                    launch_arguments['source_topic_name'] = launch_arguments['source_topic_name'].format(
                        agent['robot_vicon_name'] if launch_configuration['localisation_implementation'] == 'vicon_localisation' else get_namespace(robot_id))
                if 'source_frame_id' in launch_arguments:
                    launch_arguments['source_frame_id'] = launch_arguments['source_frame_id'].format(
                        get_namespace(robot_id))
                if 'source_child_frame_id' in launch_arguments:
                    launch_arguments['source_child_frame_id'] = launch_arguments['source_child_frame_id'].format(
                        get_namespace(robot_id))
                if 'tf_frame_id' in launch_arguments:
                    launch_arguments['tf_frame_id'] = launch_arguments['tf_frame_id'].format(
                        get_namespace(robot_id))
                if 'tf_child_frame_id' in launch_arguments:
                    launch_arguments['tf_child_frame_id'] = launch_arguments['tf_child_frame_id'].format(
                        get_namespace(robot_id))
                if 'pose_topic_name' in launch_arguments:
                    launch_arguments['pose_topic_name'] = launch_arguments['pose_topic_name'].format(
                        get_namespace(robot_id))
                if 'pose_frame_id' in launch_arguments:
                    launch_arguments['pose_frame_id'] = launch_arguments['pose_frame_id'].format(
                        get_namespace(robot_id))
                if 'pose_child_frame_id' in launch_arguments:
                    launch_arguments['pose_child_frame_id'] = launch_arguments['pose_child_frame_id'].format(
                        get_namespace(robot_id))

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
                launch_arguments=launch_arguments.items(),
            )

            result.append(_include)
    else:
        launch_arguments = {
            'robot_ids': f"{','.join([f"{agent['robot_id']}" for agent in launch_configuration['agents']])}",
            'robot_tf_prefix':
            f"{launch_configuration['manager_robot_tf_prefix']}",
                'robot_tf_suffix':
                    f"{launch_configuration['manager_robot_tf_suffix']}",
        }

        launch_arguments.update(extra_args if extra_args else {})

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
            launch_arguments=launch_arguments.items(),
        )

        result.append(_include)

    return result


def include_waypoint_controller_implementation(context) -> list[launch.Action]:
    """Include the waypoint controller implementation."""
    result = []

    launchfile = get_implementation_value(
        'waypoint_controller_implementation',
        'launchfile',
    )

    if not launchfile:
        return result

    extra_args = get_implementation_value(
        'waypoint_controller_implementation',
        'extra_args',
    )

    if not get_implementation_value('waypoint_controller_implementation', 'oneshot'):
        for agent in launch_configuration['agents']:
            robot_id = agent['robot_id']

            launch_arguments = {
                'namespace':
                    f'/{launch_configuration["manager_robot_tf_prefix"]}{robot_id}',
                'robot_id':
                    f"{robot_id}",
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


def include_mocap_implementation(context) -> list[launch.Action]:
    pass


def launch_teleop(context) -> list[launch.Action]:
    """Launch teleop nodes for robots that require it."""

    def get_namespace(i: int):
        return (
            f"{launch_configuration['manager_robot_tf_prefix']}",
            f'{i}',
            f"{launch_configuration['manager_robot_tf_suffix']}",
        )

    result = []

    for agent in launch_configuration['agents']:
        if not agent['robot_teleop']:
            continue

        robot_id = agent['robot_id']

        _teleop = launch.actions.ExecuteLocal(
            process_description=launch.descriptions.Executable(
                cmd=[
                    'xterm',
                    '-bg black -fg white -fa "Monospace" -fs 13 -title "',
                    (*get_namespace(robot_id), '/mobile_base/cmd_vel"'),
                    '-e "',
                    'ros2',
                    'run',
                    'teleop_twist_keyboard',
                    'teleop_twist_keyboard',
                    '--ros-args',
                    '-r',
                    ('__ns:=/', *get_namespace(robot_id), '/mobile_base'),
                    '-r',
                    'stamped:=true',
                    '-r',
                    (
                        'frame_id:=',
                        *get_namespace(robot_id),
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


def launch_static_transforms(context) -> list[launch.Action]:
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

    for i, transform in enumerate(launch_configuration['static_agent_transforms']):
        for agent in launch_configuration['agents']:
            arguments = []
            for k, v in transform.items():
                arguments.append(f'--{k}')
                if 'frame' in k:
                    arguments.append(
                        f"{v.format(launch_configuration['manager_robot_tf_prefix'] + str(agent['robot_id']) + launch_configuration['manager_robot_tf_suffix'])}"
                    )
                elif k == 'x':
                    arguments.append(
                        f"{v.format(agent['robot_xpos'])}"
                    )
                elif k == 'y':
                    arguments.append(
                        f"{v.format(agent['robot_ypos'])}"
                    )
                elif k == 'yaw':
                    arguments.append(
                        f"{v.format(agent['robot_theta'])}"
                    )
                else:
                    arguments.append(v)

            _static_transform = launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_agent_transform_{i}',
                output='screen',
                arguments=arguments,
            )

            result.append(_static_transform)

    return result


def setup_launch(context):
    """Generate the main launch description."""
    actions = include_epuck_implementation(context) + \
        include_comms_manager_implementation(context) + \
        include_agent_comms_implementations(context) + \
        include_waypoint_controller_implementation(context) + \
        include_localisation_implementation(context) + \
        launch_static_transforms(context) + \
        launch_teleop(context)

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription([
        launch.actions.OpaqueFunction(function=setup_launch)
    ])
