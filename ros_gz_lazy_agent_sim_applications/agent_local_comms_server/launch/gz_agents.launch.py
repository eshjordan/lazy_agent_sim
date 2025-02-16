from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FileContent
import launch
import launch_ros.actions


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "manager_server_host": "127.0.0.1",
                "manager_server_port": "50000",
                "manager_threshold_dist": "0.1",
                "manager_robot_tf_prefix": "epuck2_robot_",
                "manager_robot_tf_suffix": "",
                "manager_robot_tf_frame": "/base_link",
                "robot0_comms_host": "127.0.0.1",
                "robot0_comms_request_port": "50001",
                "robot0_knowledge_host": "aa:bb:cc:dd:ee:00",
                "robot0_knowledge_exchange_port": "1",
                "robot1_comms_host": "127.0.0.1",
                "robot1_comms_request_port": "50002",
                "robot1_knowledge_host": "aa:bb:cc:dd:ee:01",
                "robot1_knowledge_exchange_port": "1",
                "robot2_comms_host": "127.0.0.1",
                "robot2_comms_request_port": "50003",
                "robot2_knowledge_host": "aa:bb:cc:dd:ee:02",
                "robot2_knowledge_exchange_port": "1",
                "robot3_comms_host": "127.0.0.1",
                "robot3_comms_request_port": "50004",
                "robot3_knowledge_host": "aa:bb:cc:dd:ee:03",
                "robot3_knowledge_exchange_port": "1",
            }.items(),
        )
    )

    def get_namespace(id: int):
        return (
            launch.substitutions.LaunchConfiguration("manager_robot_tf_prefix"),
            f"{id}",
            launch.substitutions.LaunchConfiguration("manager_robot_tf_suffix"),
        )

    def launch_robot_comms(
        i: int, robot_id: int, node: bool = True, teleop: bool = True
    ) -> list[launch.Action]:
        _node = launch_ros.actions.Node(
            package="agent_local_comms_server",
            executable="gz_agent_comms",
            name=f"knowledge_comms_robot{i}",
            output="screen",
            # prefix=[
            #     # Debugging with gdb
            #     "xterm -bg black -fg white -fa 'Monospace' -fs 13 -e gdb -ex start --args"
            # ],
            parameters=[
                {
                    "robot_id": robot_id,
                    "manager_host": launch.substitutions.LaunchConfiguration(
                        "manager_server_host"
                    ),
                    "manager_port": launch.substitutions.LaunchConfiguration(
                        "manager_server_port"
                    ),
                    "robot_comms_host": launch.substitutions.LaunchConfiguration(
                        f"robot{i}_comms_host"
                    ),
                    "robot_comms_request_port": launch.substitutions.LaunchConfiguration(
                        f"robot{i}_comms_request_port"
                    ),
                    "robot_knowledge_host": launch.substitutions.LaunchConfiguration(
                        f"robot{i}_knowledge_host"
                    ),
                    "robot_knowledge_exchange_port": launch.substitutions.LaunchConfiguration(
                        f"robot{i}_knowledge_exchange_port"
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
                    "xterm",
                    '-bg black -fg white -fa "Monospace" -fs 13 -title "',
                    (*get_namespace(i), '/mobile_base/cmd_vel"'),
                    '-e "',
                    "ros2",
                    "run",
                    "teleop_twist_keyboard",
                    "teleop_twist_keyboard",
                    "--ros-args",
                    "-r",
                    ("__ns:=/", *get_namespace(i), "/mobile_base"),
                    "-r",
                    "stamped:=true",
                    "-r",
                    (
                        "frame_id:=",
                        *get_namespace(i),
                        launch.substitutions.LaunchConfiguration(
                            "manager_robot_tf_frame"
                        ),
                    ),
                    '"',
                ],
            ),
            shell=True,
        )

        robot_description = launch_ros.actions.Node(
            namespace=[*get_namespace(i)],
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"epuck_state_publisher_{i}",
            parameters=[
                {"use_sim_time": True},
                {
                    "frame_prefix": [
                        *get_namespace(i),
                        launch.substitutions.LaunchConfiguration(
                            "manager_robot_tf_frame"
                        ),
                        "/",
                    ],
                    "publish_frequency": 60.0,
                    "robot_description": FileContent(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare(
                                        "ros_gz_lazy_agent_sim_description"
                                    ),
                                    "models",
                                    "epuck2",
                                    "epuck2.urdf",
                                ]
                            ),
                        ]
                    ),
                },
            ],
        )

        base_link_tf = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=[
                "--frame-id",
                (
                    *get_namespace(i),
                    launch.substitutions.LaunchConfiguration("manager_robot_tf_frame"),
                ),
                "--child-frame-id",
                (
                    *get_namespace(i),
                    launch.substitutions.LaunchConfiguration("manager_robot_tf_frame"),
                    launch.substitutions.LaunchConfiguration("manager_robot_tf_frame"),
                ),
            ],
        )

        left_wheel_tf = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=[
                "--frame-id",
                (
                    *get_namespace(i),
                    "/left_wheel",
                ),
                "--child-frame-id",
                (
                    *get_namespace(i),
                    launch.substitutions.LaunchConfiguration("manager_robot_tf_frame"),
                    "/left_wheel",
                ),
            ],
        )

        right_wheel_tf = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=[
                "--frame-id",
                (
                    *get_namespace(i),
                    "/right_wheel",
                ),
                "--child-frame-id",
                (
                    *get_namespace(i),
                    launch.substitutions.LaunchConfiguration("manager_robot_tf_frame"),
                    "/right_wheel",
                ),
            ],
        )

        result = []

        if node:
            result.append(_node)
        if teleop:
            result.append(_teleop)
        result.append(robot_description)
        result.append(base_link_tf)
        result.append(left_wheel_tf)
        result.append(right_wheel_tf)

        return result

    ld = launch.LaunchDescription(
        launch_args
        + launch_robot_comms(0, 0)
        + launch_robot_comms(1, 1)
        + launch_robot_comms(2, 2)
        + launch_robot_comms(3, 3)
        + [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agent_local_comms_server"),
                            "launch",
                            "agent_local_comms_server.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "server_host": launch.substitutions.LaunchConfiguration(
                        "manager_server_host"
                    ),
                    "server_port": launch.substitutions.LaunchConfiguration(
                        "manager_server_port"
                    ),
                    "threshold_dist": launch.substitutions.LaunchConfiguration(
                        "manager_threshold_dist"
                    ),
                    "robot_tf_prefix": launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_prefix"
                    ),
                    "robot_tf_suffix": launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_suffix"
                    ),
                    "robot_tf_frame": launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"
                    ),
                    "remap_ids/0": "0",
                    "remap_ids/1": "1",
                    "remap_ids/2": "2",
                    "remap_ids/3": "3",
                }.items(),
            ),
            # launch_ros.actions.Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="static_transform_publisher",
            #     output="screen",
            #     arguments=[
            #         "--frame-id",
            #         "world",
            #         "--child-frame-id",
            #         "arena",
            #     ],
            # ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
