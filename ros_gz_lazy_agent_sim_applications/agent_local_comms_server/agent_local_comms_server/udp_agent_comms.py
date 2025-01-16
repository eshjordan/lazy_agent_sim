#!/usr/bin/env python3

import rclpy
import rclpy.node
import socket
import socketserver
import threading
import time
from typing import Callable, override


from agent_local_comms_server.packets import EpuckKnowledgePacket, EpuckNeighbourPacket
from agent_local_comms_server.robot_comms_model import (
    RobotCommsModel,
    BaseKnowledgeServer,
    BaseKnowledgeClient,
)


class UDPKnowledgeServer(BaseKnowledgeServer):
    def __init__(self, robot_model: RobotCommsModel):
        super().__init__(robot_model)

        self.server = socketserver.ThreadingUDPServer(
            (robot_model.robot_host, robot_model.robot_port), self.server_factory()
        )

        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.daemon = True

    @override
    def start(self):
        self.thread.start()

    @override
    def stop(self):
        self.server.shutdown()
        self.thread.join()

    def server_factory(self):
        robot_model = self.robot_model

        class ReceiveKnowledgeHandler(socketserver.BaseRequestHandler):
            @override
            def handle(self):
                self.request: tuple[bytes, socket.socket]
                data, client = self.request

                host = self.client_address[0]
                port = self.client_address[1]

                request = EpuckKnowledgePacket.unpack(data)

                other_known_ids = set(request.known_ids)
                difference = other_known_ids.difference(robot_model.known_ids)
                robot_model.known_ids.update(other_known_ids)

                if len(difference) > 0:
                    robot_model.logger.info(
                        f"Received new IDs from ({host}:{port}): {difference}"
                    )

                robot_model.logger.debug(
                    f"Received knowledge from {request.robot_id} ({host}:{port}): {other_known_ids}"
                )

                knowledge = EpuckKnowledgePacket(
                    robot_id=robot_model.robot_id,
                    N=len(robot_model.known_ids),
                    known_ids=list(robot_model.known_ids),
                )

                client.sendto(knowledge.pack(), self.client_address)

                robot_model.logger.debug(
                    f"Sending knowledge to {request.robot_id} ({host}:{port}): {robot_model.known_ids}"
                )

                return

        return ReceiveKnowledgeHandler


class UDPKnowledgeClient(BaseKnowledgeClient):
    def __init__(
        self,
        neighbour: EpuckNeighbourPacket,
        running: Callable[[], bool],
        robot_model: RobotCommsModel,
    ):
        super().__init__(neighbour, running, robot_model)

        self.client: socket.socket = None
        self.thread = threading.Thread(target=self.send_knowledge)
        self.thread.daemon = True

    @override
    def start(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.thread.start()

    @override
    def stop(self):
        self.client = None
        self.thread.join()

    def send_knowledge(self):
        self.robot_model.logger.info(
            f"Starting knowledge connection with {self.neighbour.robot_id} ({self.neighbour.host}:{self.neighbour.port})"
        )
        while self.running():
            knowledge = EpuckKnowledgePacket(
                robot_id=self.robot_model.robot_id,
                N=len(self.robot_model.known_ids),
                known_ids=list(self.robot_model.known_ids),
            )

            self.client.sendto(
                knowledge.pack(), (self.neighbour.host, self.neighbour.port)
            )

            self.robot_model.logger.debug(
                f"Sending knowledge to {self.neighbour.robot_id} ({self.neighbour.host}:{self.neighbour.port}): {self.robot_model.known_ids}"
            )

            data, retaddr = self.client.recvfrom(EpuckKnowledgePacket.calcsize())
            if len(data) == 0:
                self.robot_model.logger.warning(
                    f"({self.neighbour.host}:{self.neighbour.port}) disconnected"
                )
                break

            response = EpuckKnowledgePacket.unpack(data)

            other_known_ids = set(response.known_ids)
            difference = other_known_ids.difference(self.robot_model.known_ids)
            self.robot_model.known_ids.update(other_known_ids)

            if len(difference) > 0:
                self.robot_model.logger.info(
                    f"Received new IDs from ({self.neighbour.host}:{self.neighbour.port}): {difference}"
                )

            self.robot_model.logger.debug(
                f"Received knowledge from {self.neighbour.robot_id} ({self.neighbour.host}:{self.neighbour.port}): {other_known_ids}"
            )

            time.sleep(1)

        self.robot_model.logger.info(
            f"Stopping knowledge connection with {self.neighbour.robot_id} ({self.neighbour.host}:{self.neighbour.port})"
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
    robot_host = (
        node.declare_parameter("robot_host", "127.0.0.1")
        .get_parameter_value()
        .string_value
    )
    robot_port = (
        node.declare_parameter("robot_port", 50001).get_parameter_value().integer_value
    )
    logger = node.get_logger()
    robot_model = RobotCommsModel(
        robot_id,
        manager_host,
        manager_port,
        robot_host,
        robot_port,
        UDPKnowledgeServer,
        UDPKnowledgeClient,
        logger,
    )
    robot_model.start()
    rclpy.spin(node)
    robot_model.stop()


if __name__ == "__main__":
    main()
