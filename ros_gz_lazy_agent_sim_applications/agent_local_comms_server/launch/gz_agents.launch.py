from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
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
                "manager_robot_tf_frame": "",
                "robot0_host": "aa:bb:cc:dd:ee:00",
                "robot0_port": "0",
                "robot1_host": "aa:bb:cc:dd:ee:01",
                "robot1_port": "0",
                "robot2_host": "127.0.0.1",
                "robot2_port": "50004",
                "robot3_host": "127.0.0.1",
                "robot3_port": "50005",
            }.items(),
        )
    )

    def get_namespace(id: int):
        return (
            launch.substitutions.LaunchConfiguration("manager_robot_tf_prefix"),
            f"{id}",
            launch.substitutions.LaunchConfiguration("manager_robot_tf_suffix"),
        )

    def launch_robot_comms(id: int, teleop: bool = True):
        node = launch_ros.actions.Node(
            package="agent_local_comms_server",
            executable="gz_agent_comms",
            name=f"knowledge_comms_robot{id}",
            output="screen",
            parameters=[
                {
                    "robot_id": id,
                    "manager_host": launch.substitutions.LaunchConfiguration(
                        "manager_server_host"
                    ),
                    "manager_port": launch.substitutions.LaunchConfiguration(
                        "manager_server_port"
                    ),
                    "robot_host": launch.substitutions.LaunchConfiguration(
                        f"robot{id}_host"
                    ),
                    "robot_port": launch.substitutions.LaunchConfiguration(
                        f"robot{id}_port"
                    ),
                }
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                "debug",
            ],
        )

        _teleop = launch.actions.ExecuteLocal(
            process_description=launch.descriptions.Executable(
                cmd=[
                    "xterm",
                    '-bg black -fg white -fa "Monospace" -fs 13 -title "',
                    (*get_namespace(id), '/mobile_base/cmd_vel"'),
                    '-e "',
                    "ros2",
                    "run",
                    "teleop_twist_keyboard",
                    "teleop_twist_keyboard",
                    "--ros-args",
                    "-r",
                    ("__ns:=/", *get_namespace(id), "/mobile_base"),
                    "-r",
                    "stamped:=true",
                    "-r",
                    (
                        "frame_id:=",
                        *get_namespace(id),
                        "/",
                        launch.substitutions.LaunchConfiguration(
                            "manager_robot_tf_frame"
                        ),
                    ),
                    '"',
                ],
            ),
            shell=True,
        )

        global_tf = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=[
                "--frame-id",
                "world",
                "--child-frame-id",
                (*get_namespace(id), "/odom"),
            ],
        )

        return [node, _teleop, global_tf] if teleop else [node, global_tf]

    ld = launch.LaunchDescription(
        launch_args
        + launch_robot_comms(0, teleop=False)
        + launch_robot_comms(1, teleop=False)
        # + launch_robot_comms(2, teleop=False)
        # + launch_robot_comms(3, teleop=False)
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
                }.items(),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
