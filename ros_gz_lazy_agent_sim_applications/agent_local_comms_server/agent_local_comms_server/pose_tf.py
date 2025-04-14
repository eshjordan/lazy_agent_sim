import math
import select
import socket
import struct
from typing import override

import numpy as np
import rclpy
import rclpy.time
import rclpy.duration
import rclpy.node
from agent_local_comms_server.packets import (
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
    EpuckNeighbourPacket,
    EpuckKnowledgePacket,
)
from socketserver import BaseRequestHandler, ThreadingUDPServer
import threading
import tf2_ros.buffer
import tf2_ros.transform_listener
from lazy_agent_sim_interfaces.msg import (
    EpuckKnowledgePacket as EpuckKnowledgePacketMsg,
    EpuckKnowledgeRecord as EpuckKnowledgeRecordMsg,
    Boundary as BoundaryMsg,
    Centroid as CentroidMsg,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster

class FramePublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('pose_tf')

        self.declare_parameter('topic_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('frame_id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('child_frame_id', rclpy.Parameter.Type.STRING)


        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            geometry_msgs.msg.PoseStamped,
            self.get_parameter('topic_name').value,
            self.handle_pose_callback,
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
        )
        self.subscription  # prevent unused variable warning

    def handle_pose_callback(self, msg: geometry_msgs.msg.PoseStamped):
        t = geometry_msgs.msg.TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('frame_id').value
        t.child_frame_id = self.get_parameter('child_frame_id').value

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation = msg.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
