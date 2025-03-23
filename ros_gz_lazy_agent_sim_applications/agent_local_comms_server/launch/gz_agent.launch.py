from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FileContent
import launch
import launch_ros.actions


def launch_robot_comms(
    debug: bool = False,
) -> list[launch.Action]:
    # breakpoint()  # launch_robot_comms

    _node = launch_ros.actions.Node(
        package="agent_local_comms_server",
        executable="gz_agent_comms",
        name=f"knowledge_comms_robot",
        output="screen",
        prefix=(
            None
            if not debug
            else [
                # Debugging with pdb
                "xterm -bg black -fg white -fa 'Monospace' -fs 13 -e python3 -m pdb"
            ]
        ),
        parameters=[
            {
                "robot_id": launch.substitutions.LaunchConfiguration(
                    "robot_id"
                ),
                "manager_host": launch.substitutions.LaunchConfiguration(
                    "manager_server_host"
                ),
                "manager_port": launch.substitutions.LaunchConfiguration(
                    "manager_server_port"
                ),
                "robot_comms_host": launch.substitutions.LaunchConfiguration(
                    "robot_comms_host"
                ),
                "robot_comms_request_port": launch.substitutions.LaunchConfiguration(
                    "robot_comms_request_port"
                ),
                "robot_knowledge_host": launch.substitutions.LaunchConfiguration(
                    "robot_knowledge_host"
                ),
                "robot_knowledge_exchange_port": launch.substitutions.LaunchConfiguration(
                    "robot_knowledge_exchange_port"
                ),
            }
        ],
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
    )

    return [_node]


def generate_launch_description():
    # breakpoint()  # generate_launch_description gz_agent

    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "manager_server_host": "127.0.0.1",
                "manager_server_port": "50000",
                "robot_id": "0",
                "robot_comms_host": "127.0.0.1",
                "robot_comms_request_port": "50001",
                "robot_knowledge_host": "aa:bb:cc:dd:ee:00",
                "robot_knowledge_exchange_port": "1",
            }.items(),
        )
    )

    ld = launch.LaunchDescription(
        launch_args
        + launch_robot_comms()
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
