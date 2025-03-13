"""Launchfile for the cpp_epuck package."""

import launch
from launch.substitutions import PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the main launch description."""
    launch_args = [
        launch.actions.DeclareLaunchArgument(name=x[0], default_value=x[1])
        for x in {
            'manager_server_host': '192.168.0.2',
            'manager_server_port': '50000',
            'manager_threshold_dist': '0.1',
            'manager_robot_tf_prefix': 'epuck2_robot_',
            'manager_robot_tf_suffix': '',
            'manager_robot_tf_frame': '/base_link',
            'robot0_host': '192.168.0.2',
            'robot0_knowledge_exchange_port': '50009',
            'robot0_knowledge_request_port': '50002',
            'robot1_host': '192.168.0.2',
            'robot1_knowledge_exchange_port': '50003',
            'robot1_knowledge_request_port': '50004',
            'robot2_host': '192.168.0.2',
            'robot2_knowledge_exchange_port': '50005',
            'robot2_knowledge_request_port': '50006',
            'robot3_host': '192.168.0.2',
            'robot3_knowledge_exchange_port': '50007',
            'robot3_knowledge_request_port': '50008',
        }.items()
    ]

    def get_namespace(id_number: int):
        return (
            launch.substitutions.LaunchConfiguration(
                'manager_robot_tf_prefix'),
            f'{id_number}',
            launch.substitutions.LaunchConfiguration(
                'manager_robot_tf_suffix'),
        )

    def launch_robot_comms(
        i: int, robot_id: int, node: bool = True, teleop: bool = True
    ) -> list[launch.Action]:
        _node = launch_ros.actions.Node(
            package='cpp_epuck',
            executable='cpp_epuck',
            name=f'knowledge_comms_robot{i}',
            output='screen',
            parameters=[
                {
                    'robot_id': robot_id,
                    'manager_host': launch.substitutions.LaunchConfiguration(
                        'manager_server_host'
                    ),
                    'manager_port': launch.substitutions.LaunchConfiguration(
                        'manager_server_port'
                    ),
                    'robot_host': launch.substitutions.LaunchConfiguration(
                        f'robot{i}_host'
                    ),
                    'robot_knowledge_exchange_port': launch.substitutions
                    .LaunchConfiguration(
                        f'robot{i}_knowledge_exchange_port'
                    ),
                    'robot_knowledge_request_port': launch.substitutions
                    .LaunchConfiguration(
                        f'robot{i}_knowledge_request_port'
                    ),
                }
            ],
            # arguments=[
            #     "--ros-args",
            #     "--log-level",
            #     "debug",
            # ],
        )

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
                        launch.substitutions.LaunchConfiguration(
                            'manager_robot_tf_frame'
                        ),
                    ),
                    '"',
                ],
            ),
            shell=True,
        )

        result = []

        if node:
            result.append(_node)
        if teleop:
            result.append(_teleop)

        return result

    ld = launch.LaunchDescription(
        launch_args
        + launch_robot_comms(0, 0, teleop=True)
        + launch_robot_comms(1, 1, teleop=True)
        # + launch_robot_comms(2, 2, teleop=False)
        # + launch_robot_comms(3, 3, teleop=False)
        + [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources
                .PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('agent_local_comms_server'),
                            'launch',
                            'agent_local_comms_server.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'server_host': launch.substitutions.LaunchConfiguration(
                        'manager_server_host'
                    ),
                    'server_port': launch.substitutions.LaunchConfiguration(
                        'manager_server_port'
                    ),
                    'threshold_dist': launch.substitutions.LaunchConfiguration(
                        'manager_threshold_dist'
                    ),
                    'robot_tf_prefix': launch.substitutions
                    .LaunchConfiguration(
                        'manager_robot_tf_prefix'
                    ),
                    'robot_tf_suffix': launch.substitutions
                    .LaunchConfiguration(
                        'manager_robot_tf_suffix'
                    ),
                    'robot_tf_frame': launch.substitutions.LaunchConfiguration(
                        'manager_robot_tf_frame'
                    ),
                    'remap_ids/0': '5831',
                    'remap_ids/1': '5731',
                }.items(),
            ),
            # launch.actions.IncludeLaunchDescription(
            #     launch.launch_description_sources.PythonLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("epuck_driver_cpp"),
            #                 "launch",
            #                 "multi_epuck2.launch.py",
            #             ]
            #         )
            #     ),
            #     launch_arguments={
            #         "robot_id0": "5831",
            #         "robot_addr0": "192.168.0.22",
            #         "robot_port0": "1000",
            #         "robot_sim_en0": "false",
            #         "robot_id1": "1",
            #         "robot_addr1": "192.168.0.21",
            #         "robot_port1": "1000",
            #         "robot_sim_en1": "false",
            #         "robot_id2": "2",
            #         "robot_addr2": "192.168.0.2",
            #         "robot_port2": "10002",
            #         "robot_sim_en2": "true",
            #         "robot_id3": "3",
            #         "robot_addr3": "192.168.0.2",
            #         "robot_port3": "10003",
            #         "robot_sim_en3": "true",
            #     }.items(),
            # ),
        ]
    )
    return ld


if __name__ == '__main__':
    generate_launch_description()
