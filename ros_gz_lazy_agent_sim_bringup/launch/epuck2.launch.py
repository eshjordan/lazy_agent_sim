# ROS 2 launch file for launching the epuck2 robot in Gazebo and RViz
# Copyright (C) 2024  Jordan Esh

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.actions import IncludeLaunchDescription, GroupAction
import launch_ros
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FileContent
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_namespace(id: int):
    return (
        launch.substitutions.LaunchConfiguration(
            "manager_robot_tf_prefix"),
        f"{id}",
        launch.substitutions.LaunchConfiguration(
            "manager_robot_tf_suffix"),
    )


def setup_launch(context):
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                'agent_ids': '',
                "gz_version": "9",
                "gui": "true",
                "manager_robot_tf_prefix": "epuck2_robot_",
                "manager_robot_tf_suffix": "",
            }.items(),
        )
    ) + [
        SetEnvironmentVariable(
            "GZ_VERSION", LaunchConfiguration("gz_version")),
    ]

    # Find paths to other projects
    sim_bringup = FindPackageShare("ros_gz_lazy_agent_sim_bringup")
    sim_gazebo = FindPackageShare("ros_gz_lazy_agent_sim_gazebo")
    ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Launch Gazebo (GUI only)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"]),
        ),
        launch_arguments={"gz_args": "-g"}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    parameter_bridge = \
        ComposableNode(
            package="ros_gz_bridge",
            plugin="ros_gz_bridge::RosGzBridge",
            name="parameter_bridge",
            parameters=[
                {
                    "use_sim_time": True,
                    "expand_gz_topic_names": True,
                    "config_file": PathJoinSubstitution(
                        [sim_bringup, "config", "ros_gz_lazy_agent_sim_bridge.yaml"]
                    ),
                    "qos_overrides./tf_static.publisher.durability": "transient_local",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    # Launch Gazebo server
    gz_server = \
        ComposableNode(
            package="ros_gz_sim",
            plugin="ros_gz_sim::GzServer",
            name="gz_server",
            parameters=[
                {
                    "world_sdf_file": PathJoinSubstitution(
                        [sim_gazebo, "worlds", "epuck2.sdf"]
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    # Run the bridge and Gazebo server in a container, better communication
    gz_container = ComposableNodeContainer(
        name="gz_bridged_sim",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            parameter_bridge,
            gz_server,
        ],
        ros_arguments=['--disable-stdout-logs'],
    )

    frame_publishers = []

    def process_agent_ids(context):
        agent_ids_value = LaunchConfiguration("agent_ids").perform(context)
        return agent_ids_value.split(",") if agent_ids_value else []

    for agent_id in process_agent_ids(context):
        group = GroupAction([
            PushRosNamespace(f'epuck2_robot_{agent_id}'),
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"use_sim_time": True},
                    {
                        "frame_prefix": [
                            *get_namespace(agent_id),
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
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                arguments=[
                    '--frame-id',
                    [*get_namespace(agent_id),],
                    '--child-frame-id',
                    [*get_namespace(agent_id), '/map'],
                ],
            )
        ])

        frame_publishers.append(group)

    return launch_args + [
        gz_sim,
        gz_container,
    ] + frame_publishers


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup_launch)])
