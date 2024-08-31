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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FileContent
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
        DeclareLaunchArgument(
            "gz_version", default_value="8", description="Gazebo version."
        ),
        SetEnvironmentVariable("GZ_VERSION", LaunchConfiguration("gz_version")),
    ]

    # Find paths to other projects
    sim_bringup = FindPackageShare("ros_gz_lazy_agent_sim_bringup")
    sim_gazebo = FindPackageShare("ros_gz_lazy_agent_sim_gazebo")
    sim_description = FindPackageShare("ros_gz_lazy_agent_sim_description")
    ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Load the URDF file from "description" package
    robot_desc = FileContent(
        PathJoinSubstitution([sim_description, "models", "epuck2", "epuck2.sdf"])
    )

    # Launch Gazebo (GUI only)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"]),
        ),
        launch_arguments={"gz_args": "-g"}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    parameter_bridge = ComposableNode(
        package="ros_gz_bridge",
        plugin="ros_gz_bridge::RosGzBridge",
        # name="parameter_bridge",
        # namespace="namespace",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [sim_bringup, "config", "ros_gz_lazy_agent_sim_bridge.yaml"]
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Launch Gazebo server
    gz_server = ComposableNode(
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

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([sim_bringup, "config", "epuck2.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        launch_args
        + [
            gz_sim,
            gz_container,
            robot_state_publisher,
            rviz,
        ]
    )
