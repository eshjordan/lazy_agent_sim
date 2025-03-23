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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
import launch_ros
from launch_ros.actions import ComposableNodeContainer
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


def generate_launch_description():
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "gz_version": "9",
                "manager_robot_tf_prefix": "epuck2_robot_",
                "manager_robot_tf_suffix": "",
                "manager_robot_tf_frame": "/base_link",
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
    )

    frame_publishers = []

    for i in range(4):
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

        frame_publishers.append(robot_description)

        base_link_tf = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=[
                "--frame-id",
                (
                    *get_namespace(i),
                    launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"),
                ),
                "--child-frame-id",
                (
                    *get_namespace(i),
                    launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"),
                    launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"),
                ),
            ],
        )

        frame_publishers.append(base_link_tf)

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
                    launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"),
                    "/left_wheel",
                ),
            ],
        )

        frame_publishers.append(left_wheel_tf)

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
                    launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"),
                    "/right_wheel",
                ),
            ],
        )

        frame_publishers.append(right_wheel_tf)

    return LaunchDescription(
        launch_args
        + [
            gz_sim,
            gz_container,
        ] + frame_publishers
    )
