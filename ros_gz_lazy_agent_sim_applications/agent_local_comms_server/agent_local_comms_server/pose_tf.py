import math
import select
import socket
import struct
from typing import override

import numpy as np
import rclpy
import rclpy.clock
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
import tf2_py
from tf2_ros.buffer import Buffer
import tf2_ros.transform_listener
from lazy_agent_sim_interfaces.msg import (
    EpuckKnowledgePacket as EpuckKnowledgePacketMsg,
    EpuckKnowledgeRecord as EpuckKnowledgeRecordMsg,
    Boundary as BoundaryMsg,
    Centroid as CentroidMsg,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster, TransformListener
from tf_transformations import translation_matrix, quaternion_matrix, inverse_matrix, translation_from_matrix, quaternion_from_matrix
import time


class FramePublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('pose_tf', parameter_overrides=[
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('source_topic_name',
                               rclpy.Parameter.Type.STRING)
        self.declare_parameter('source_frame_id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('tf_frame_id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('tf_child_frame_id',
                               rclpy.Parameter.Type.STRING)
        self.declare_parameter('pose_topic_name',
                               rclpy.Parameter.Type.STRING)
        self.declare_parameter('pose_frame_id', rclpy.Parameter.Type.STRING)

        # Initialize the transform broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            # qos=QoSProfile(
            #     reliability=ReliabilityPolicy.BEST_EFFORT,
            #     durability=DurabilityPolicy.VOLATILE,
            #     history=HistoryPolicy.KEEP_LAST,
            #     depth=10
            # ),
            # static_qos=QoSProfile(
            #     reliability=ReliabilityPolicy.BEST_EFFORT,
            #     durability=DurabilityPolicy.VOLATILE,
            #     history=HistoryPolicy.KEEP_LAST,
            #     depth=10
            # ),
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish the pose to the pose topic if enabled
        if self.get_parameter('pose_topic_name').value != "":
            self.get_logger().info("CREATING PUBLISHER")
            self.publisher = self.create_publisher(
                geometry_msgs.msg.PoseStamped,
                self.get_parameter('pose_topic_name').value,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                ),
            )

        self.subscription = self.create_subscription(
            geometry_msgs.msg.PoseStamped,
            self.get_parameter('source_topic_name').value,
            self.handle_pose_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
        )
        self.subscription  # prevent unused variable warning

    def handle_pose_callback(self, msg: geometry_msgs.msg.PoseStamped):
        # Ignore the frame_id field in the msg, instead use the source_frame_id param
        # Msg is transform from source_frame_id to tf_child_frame_id
        # msg = TF(source_frame_id, tf_frame_id) @ TF(tf_frame_id, tf_child_frame_id)

        # Lookup the transform from the source_frame_id to the tf_frame_id
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('tf_frame_id').value,
                self.get_parameter('source_frame_id').value,
                rclpy.time.Time(),
            )
        except tf2_py.LookupException as e:
            self.get_logger().error(
                f"LookupException: {e}")
            return

        # Convert the pose from the message into a transformation matrix
        msg_matrix = quaternion_matrix([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]) @ translation_matrix([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Convert the transform from tf_buffer into a transformation matrix
        source_frame_id_to_tf_frame_id_matrix = quaternion_matrix([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ]) @ translation_matrix([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])

        tf_frame_id_to_tf_child_frame_id = inverse_matrix(
            source_frame_id_to_tf_frame_id_matrix) @ msg_matrix
        translation = translation_from_matrix(
            tf_frame_id_to_tf_child_frame_id)
        quaternion = quaternion_from_matrix(
            tf_frame_id_to_tf_child_frame_id)

        # Broadcast the transform from tf_frame_id to tf_child_frame_id
        timestamp = self.get_clock().now()

        child_tf = geometry_msgs.msg.TransformStamped()
        child_tf.header.stamp = timestamp.to_msg()
        child_tf.header.frame_id = self.get_parameter('tf_frame_id').value
        child_tf.child_frame_id = self.get_parameter('tf_child_frame_id').value
        child_tf.transform.translation.x = translation[0]
        child_tf.transform.translation.y = translation[1]
        child_tf.transform.translation.z = translation[2]
        child_tf.transform.rotation.x = quaternion[0]
        child_tf.transform.rotation.y = quaternion[1]
        child_tf.transform.rotation.z = quaternion[2]
        child_tf.transform.rotation.w = quaternion[3]

        # Send the transformation
        self.tf_buffer.set_transform(child_tf, 'default_authority')
        self.tf_broadcaster.sendTransform(child_tf)

        # Publish the pose to the pose topic if enabled
        if self.get_parameter('pose_topic_name').value != "":
            try:
                t = self.tf_buffer.lookup_transform(
                    self.get_parameter('tf_child_frame_id').value,
                    self.get_parameter('pose_frame_id').value,
                    timestamp,
                )
                # t = self.tf_buffer.lookup_transform_full(
                #     self.get_parameter('tf_child_frame_id').value,
                #     rclpy.time.Time(),
                #     self.get_parameter('pose_frame_id').value,
                #     rclpy.time.Time(),
                #     self.get_parameter('tf_child_frame_id').value,
                # )
            except tf2_py.LookupException as e:
                self.get_logger().error(
                    f"LookupException: {e}", throttle_duration_sec=2.0)
                return
            except tf2_py.ConnectivityException as e:
                self.get_logger().error(
                    f"ConnectivityException: {e}", throttle_duration_sec=2.0)
                return
            except tf2_py.ExtrapolationException as e:
                self.get_logger().error(
                    f"ExtrapolationException: {e}", throttle_duration_sec=2.0)
                return

            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.header.stamp = timestamp.to_msg()
            pose_msg.header.frame_id = self.get_parameter(
                'pose_frame_id').value
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation.x = t.transform.rotation.x
            pose_msg.pose.orientation.y = t.transform.rotation.y
            pose_msg.pose.orientation.z = t.transform.rotation.z
            pose_msg.pose.orientation.w = t.transform.rotation.w

            # Publish the pose
            self.publisher.publish(pose_msg)


def main():
    rclpy.init()
    node = FramePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
