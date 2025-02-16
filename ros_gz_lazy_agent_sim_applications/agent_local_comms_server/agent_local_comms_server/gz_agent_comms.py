#!/usr/bin/env python3

from gz.msgs10.dataframe_pb2 import Dataframe
from gz.transport13 import Node

import rclpy
import rclpy.node
import logging
import threading
import time
from typing import Callable, override
import queue

from agent_local_comms_server.packets import (
    EpuckKnowledgePacket,
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
        self.node = None
        self.pub_knowledge = None

    @override
    def start(self):
        self.node = Node()
        self.pub_knowledge = self.node.advertise("/broker/msgs", Dataframe)

        if not self.node.subscribe(
            Dataframe, f"{self.robot_model.robot_knowledge_host}/rx", self.handle
        ):
            raise RuntimeError(
                f"Failed to subscribe to topic {self.robot_model.robot_knowledge_host}/rx"
            )

    @override
    def stop(self):
        del self.pub_knowledge
        del self.node

    def handle(self, msg: Dataframe):
        address = msg.src_address

        request = EpuckKnowledgePacket.unpack(msg.data)

        if request.robot_id < self.robot_model.robot_id:
            return

        other_known_ids = set(request.known_ids)
        difference = other_known_ids.difference(self.robot_model.known_ids)
        self.robot_model.known_ids.update(other_known_ids)

        if len(difference) > 0:
            self.robot_model.logger.info(
                f"Received new IDs from ({address}): {difference}"
            )

        self.robot_model.logger.info(
            f"Server: Received knowledge from {request.robot_id} ({address}): {other_known_ids}"
        )

        knowledge = EpuckKnowledgePacket(
            robot_id=self.robot_model.robot_id,
            seq=self.robot_model.get_seq(),
            N=len(self.robot_model.known_ids),
            known_ids=list(self.robot_model.known_ids),
        )

        response = Dataframe()
        response.data = knowledge.pack()
        response.src_address = self.robot_model.robot_knowledge_host
        response.dst_address = address

        self.pub_knowledge.publish(response)

        self.robot_model.logger.debug(
            f"Server: Sending knowledge to {request.robot_id} ({address}): {self.robot_model.known_ids}"
        )

        return


class GZKnowledgeClient(BaseKnowledgeClient):
    def __init__(
        self,
        neighbour: EpuckNeighbourPacket,
        running: Callable[[], bool],
        robot_model: RobotCommsModel,
    ):
        super().__init__(neighbour, running, robot_model)
        self.node = None
        self.pub_knowledge = None
        self.thread = threading.Thread(target=self.send_knowledge)
        self.stop_event = threading.Event()

    @override
    def start(self):
        self.node = Node()
        self.pub_knowledge = self.node.advertise("/broker/msgs", Dataframe)
        self.stop_event.clear()
        self.thread.start()

    @override
    def stop(self):
        self.stop_event.set()
        self.thread.join()
        del self.pub_knowledge
        del self.node

    def send_knowledge(self):
        self.robot_model.logger.info(
            f"Client: Starting knowledge connection with {self.neighbour.robot_id} ({self.neighbour.host})"
        )

        response_queue = queue.SimpleQueue()

        def handle(msg: Dataframe):
            if msg.src_address == self.neighbour.host:
                response_queue.put(msg)

        if not self.node.subscribe(
            Dataframe, f"{self.robot_model.robot_knowledge_host}/rx", handle
        ):
            raise RuntimeError(
                f"Failed to subscribe to topic {self.robot_model.robot_knowledge_host}/rx"
            )

        while self.running() and not self.stop_event.is_set():
            knowledge = EpuckKnowledgePacket(
                robot_id=self.robot_model.robot_id,
                seq=self.robot_model.get_seq(),
                N=len(self.robot_model.known_ids),
                known_ids=list(self.robot_model.known_ids),
            )

            self.robot_model.logger.info(
                f"Client: Sending knowledge to {self.neighbour.robot_id} ({self.neighbour.host}): {self.robot_model.known_ids}"
            )

            request = Dataframe()
            request.data = knowledge.pack()
            request.src_address = self.robot_model.robot_knowledge_host
            request.dst_address = self.neighbour.host

            self.pub_knowledge.publish(request)

            try:
                response: Dataframe = response_queue.get(timeout=2.0)
            except queue.Empty:
                self.robot_model.logger.warning(
                    f"Failed to receive knowledge from {self.neighbour.robot_id} ({self.neighbour.host})"
                )
                break

            response = EpuckKnowledgePacket.unpack(response.data)

            other_known_ids = set(response.known_ids)
            difference = other_known_ids.difference(self.robot_model.known_ids)
            self.robot_model.known_ids.update(other_known_ids)

            if len(difference) > 0:
                self.robot_model.logger.info(
                    f"Received new IDs from ({self.neighbour.host}): {difference}"
                )

            self.robot_model.logger.debug(
                f"Client: Received knowledge from {self.neighbour.robot_id} ({self.neighbour.host}): {other_known_ids}"
            )

            time.sleep(1)

        self.robot_model.logger.info(
            f"Stopping knowledge connection with {self.neighbour.robot_id} ({self.neighbour.host})"
        )


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
    robot_comms_host = (
        node.declare_parameter("robot_comms_host", "127.0.0.1")
        .get_parameter_value()
        .string_value
    )
    robot_comms_request_port = (
        node.declare_parameter("robot_comms_request_port", 50001)
        .get_parameter_value()
        .integer_value
    )
    robot_knowledge_host = (
        node.declare_parameter("robot_knowledge_host", "aa:bb:cc:dd:ee:00")
        .get_parameter_value()
        .string_value
    )
    robot_knowledge_exchange_port = (
        node.declare_parameter("robot_knowledge_exchange_port", 0)
        .get_parameter_value()
        .integer_value
    )
    logger = node.get_logger()
    robot_model = RobotCommsModel(
        robot_id,
        manager_host,
        manager_port,
        robot_comms_host,
        robot_comms_request_port,
        robot_knowledge_host,
        robot_knowledge_exchange_port,
        GZKnowledgeServer,
        GZKnowledgeClient,
        logger,
    )
    robot_model.start()
    rclpy.spin(node)
    robot_model.stop()


if __name__ == "__main__":
    main()
