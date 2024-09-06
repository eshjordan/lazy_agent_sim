from gz.msgs10.dataframe_pb2 import Dataframe
from gz.transport13 import Node

import logging
import time

from agent_local_comms_server.packets import (
    EpuckAddressKnowledgePacket,
    EpuckNeighbourPacket,
)
from agent_local_comms_server.robot_comms_model import (
    RobotCommsModel,
    BaseKnowledgeServer,
    BaseKnowledgeClient,
)


class GZKnowledgeServer(BaseKnowledgeServer):
    def __init__(self, robot_model: RobotCommsModel):
        super().__init__(robot_model)
        self.node = Node()
        self.pub_knowledge = None
        self.sub_knowledge = None

    def start(self):
        self.pub_knowledge = self.node.advertise("/broker/msgs", Dataframe)

        if !self.node.subscribe(Dataframe, f"{robot_model.robot_host}/rx", self.handle):
            raise RuntimeError(f"Failed to subscribe to topic {robot_model.robot_host}/rx")

    def stop(self):
        del self.sub_knowledge
        del self.pub_knowledge

    def handle(self, msg: Dataframe):
        self.robot_model.logger.info("Received knowledge:")
        self.robot_model.logger.info(msg.header)
        self.robot_model.logger.info(msg.src_address)
        self.robot_model.logger.info(msg.dst_address)
        self.robot_model.logger.info(msg.data)

        address = msg.src_address

        request = EpuckKnowledgePacket.unpack(msg.data)

        other_known_ids = set(request.known_ids)
        difference = other_known_ids.difference(robot_model.known_ids)
        robot_model.known_ids.update(other_known_ids)

        if len(difference) > 0:
            robot_model.logger.info(
                f"Received new IDs from ({address}): {difference}"
            )

        robot_model.logger.debug(
            f"Received knowledge from {request.robot_id} ({address}): {other_known_ids}"
        )

        knowledge = EpuckKnowledgePacket(
            robot_id=robot_model.robot_id,
            N=len(robot_model.known_ids),
            known_ids=list(robot_model.known_ids),
        )

        response = StringMsg()
        response.data = knowledge.pack()
        response.set_src_address(robot_model.robot_host)
        response.set_dst_address(address)

        self.pub_knowledge.publish(response)

        client.sendto(knowledge.pack(), self.client_address)

        robot_model.logger.debug(
            f"Sending knowledge to {request.robot_id} ({address}): {robot_model.known_ids}"
        )

        return


def main2():
    node = Node()
    addr = "epuck2_0_bt_addr"
    subscription_topic = "epuck2_0_bt_addr/rx"

    vector3d_msg = Vector3d()
    vector3d_msg.x = 10
    vector3d_msg.y = 15
    vector3d_msg.z = 20

    stringmsg_msg = StringMsg()
    stringmsg_msg.data = "Hello"
    try:
        count = 0
        while True:
            count += 1
            vector3d_msg.x = count
            if not pub_stringmsg.publish(stringmsg_msg):
                break
            print("Publishing 'Hello' on topic [{}]".format(stringmsg_topic))
            if not pub_vector3d.publish(vector3d_msg):
                break
            print("Publishing a Vector3d on topic [{}]".format(vector3d_topic))
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass


def main():
    rclpy.init()
    node = rclpy.node.Node("robot_comms_model")
    robot_id = node.declare_parameter("robot_id", 0).get_parameter_value().integer_value
    manager_host = (
        node.declare_parameter("manager_host", "127.0.0.1")
        .get_parameter_value()
        .string_value
    )
    manager_port = (
        node.declare_parameter("manager_port", 50000)
        .get_parameter_value()
        .integer_value
    )
    robot_host = (
        node.declare_parameter("robot_host", "8a:7a:37:7e:67:54")
        .get_parameter_value()
        .string_value
    )
    robot_port = (
        node.declare_parameter("robot_port", 0).get_parameter_value().integer_value
    )
    logger = node.get_logger()
    robot_model = RobotCommsModel(
        robot_id,
        manager_host,
        manager_port,
        robot_host,
        robot_port,
        GZKnowledgeServer,
        GZKnowledgeClient,
        logger,
    )
    robot_model.start()
    rclpy.spin(node)
    robot_model.stop()


if __name__ == "__main__":
    main()
