#!/usr/bin/env python3

import select
import socket
import threading
import logging
import time
from typing import Callable

from agent_local_comms_server.packets import (
    EpuckKnowledgePacket,
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
    EpuckNeighbourPacket,
)


class BaseKnowledgeServer:
    def __init__(self, robot_model: "RobotCommsModel"):
        self.robot_model = robot_model

    def start(self):
        pass

    def stop(self):
        pass


class BaseKnowledgeClient:
    def __init__(
        self,
        neighbour: EpuckNeighbourPacket,
        running: Callable[[], bool],
        robot_model: "RobotCommsModel",
    ):
        self.neighbour = neighbour
        self.running = running
        self.robot_model = robot_model

    def start(self):
        pass

    def stop(self):
        pass


class RobotCommsModel:
    def __init__(
        self,
        robot_id: int,
        manager_host: str,
        manager_port: int,
        robot_host: str,
        robot_port: int,
        KnowledgeServerClass: type[BaseKnowledgeServer],
        KnowledgeClientClass: type[BaseKnowledgeClient],
        logger=logging.getLogger(__name__),
    ):
        self.robot_id = robot_id
        self.manager_host = manager_host
        self.manager_port = manager_port
        self.robot_host = robot_host
        self.robot_port = robot_port
        self.KnowledgeServerClass = KnowledgeServerClass
        self.KnowledgeClientClass = KnowledgeClientClass
        self.logger = logger

        self.known_ids = set([self.robot_id])

        self.heartbeat_thread = None
        self.heartbeat_client: socket.socket = None

        self.knowledge_server: BaseKnowledgeServer = None
        self.knowledge_clients: dict[int, BaseKnowledgeClient] = None

    def __del__(self):
        self.stop()
        super().__del__()

    def start(self):
        self.heartbeat_thread = threading.Thread(target=self.exchange_heartbeats)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

    def exchange_heartbeats(self):
        self.logger.info(
            f"Starting robot comms model on {self.robot_host}:{self.robot_port}"
        )

        self.knowledge_server = self.KnowledgeServerClass(self)
        self.knowledge_server.start()

        self.heartbeat_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.knowledge_clients = {}

        while True:
            self.logger.debug(
                f"Sending heartbeat to {self.manager_host}:{self.manager_port}"
            )
            self.heartbeat_client.sendto(
                EpuckHeartbeatPacket(
                    robot_id=self.robot_id,
                    robot_host=self.robot_host,
                    robot_port=self.robot_port,
                ).pack(),
                (self.manager_host, self.manager_port),
            )

            data_to_read = select.select([self.heartbeat_client], [], [], 0.01)[0]
            if not data_to_read:
                continue

            data, retaddr = self.heartbeat_client.recvfrom(
                EpuckHeartbeatResponsePacket.calcsize()
            )
            response = EpuckHeartbeatResponsePacket.unpack(data)

            for neighbour in response.neighbours:
                self.logger.debug(
                    f"Received neighbour: {neighbour.robot_id} ({neighbour.host}:{neighbour.port}) at distance {neighbour.dist}"
                )

            # Start threads for new neighbours
            new_in_range_neighbours = [
                neighbour
                for neighbour in response.neighbours
                if neighbour.robot_id not in self.knowledge_clients.keys()
                and neighbour.robot_id < self.robot_id
            ]

            for neighbour in new_in_range_neighbours:
                self.logger.info(
                    f"Starting thread for neighbour {neighbour.robot_id} ({neighbour.host}:{neighbour.port})"
                )
                client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

                self.knowledge_clients[neighbour.robot_id] = self.KnowledgeClientClass(
                    neighbour,
                    lambda: neighbour.robot_id in self.knowledge_clients.keys(),
                    self,
                )

                self.knowledge_clients[neighbour.robot_id].start()

            # Stop threads for out of range neighbours
            out_of_range_neighbour_ids = [
                neighbour_id
                for neighbour_id in self.knowledge_clients.keys()
                if neighbour_id
                not in [neighbour.robot_id for neighbour in response.neighbours]
            ]

            for neighbour_id in out_of_range_neighbour_ids:
                self.logger.info(
                    f"Stopping thread for neighbour {neighbour.robot_id} ({neighbour.host}:{neighbour.port})"
                )
                knowledge_client = self.knowledge_clients.pop(neighbour_id)
                knowledge_client.stop()

            time.sleep(1)

    def stop(self):
        self.heartbeat_client.shutdown(socket.SHUT_RDWR)
        for knowledge_client in self.knowledge_clients.items():
            knowledge_client.stop()
        self.knowledge_server.stop()
