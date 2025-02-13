import struct
from dataclasses import dataclass

ENDIAN_FMT = "<"

MAX_ROBOTS = 10
MAX_HOST_LEN = 18

ROBOT_ID_TYPE_FMT_STR: str = "H"  # ushort

EPUCK_HEARTBEAT_PACKET_FMT_STR: str = (
    ENDIAN_FMT + f"B{ROBOT_ID_TYPE_FMT_STR}{MAX_HOST_LEN+1}sH"
)
EPUCK_HEARTBEAT_PACKET_ID: int = 0x20


@dataclass
class EpuckHeartbeatPacket:
    id: int = 0x20  # byte
    robot_id: int = 0x0  # see above
    robot_host: str = ""  # str
    robot_port: int = 0  # ushort

    def pack(self):
        return struct.pack(
            EPUCK_HEARTBEAT_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.robot_host.encode("ascii"),
            self.robot_port,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_HEARTBEAT_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_HEARTBEAT_PACKET_ID}"
            )
        args = struct.unpack(EPUCK_HEARTBEAT_PACKET_FMT_STR, buffer)
        obj = cls(*args)
        obj.robot_host = obj.robot_host.decode("ascii").rstrip("\x00")  # type: ignore
        return obj

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_HEARTBEAT_PACKET_FMT_STR)


EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR: str = ENDIAN_FMT + "BB"
EPUCK_HEARTBEAT_RESPONSE_PACKET_ID: int = 0x21


@dataclass
class EpuckHeartbeatResponsePacket:
    neighbours: list["EpuckNeighbourPacket"]  # list of EpuckNeighbourPacket
    id: int = 0x21  # byte
    num_neighbours: int = 0  # byte

    def pack(self):
        return struct.pack(
            EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR,
            self.id,
            self.num_neighbours,
        ) + b"".join([n.pack() for n in self.neighbours])

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_HEARTBEAT_RESPONSE_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_HEARTBEAT_RESPONSE_PACKET_ID}"
            )

        id, num_neighbours = struct.unpack(
            EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR, buffer[:2]
        )
        neighbours = []
        offset = 2
        for _ in range(min(num_neighbours, MAX_ROBOTS)):
            neighbour = EpuckNeighbourPacket.unpack(
                buffer[offset : offset + EpuckNeighbourPacket.calcsize()]
            )
            offset += EpuckNeighbourPacket.calcsize()
            neighbours.append(neighbour)
        return cls(id=id, num_neighbours=num_neighbours, neighbours=neighbours)

    @classmethod
    def calcsize(cls):
        return (
            struct.calcsize(EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR)
            + MAX_ROBOTS * EpuckNeighbourPacket.calcsize()
        )


EPUCK_NEIGHBOUR_PACKET_FMT_STR: str = (
    ENDIAN_FMT + f"{ROBOT_ID_TYPE_FMT_STR}{MAX_HOST_LEN+1}sHf"
)


@dataclass
class EpuckNeighbourPacket:
    robot_id: int = 0  # see above
    host: str = ""  # str
    port: int = 0  # ushort
    dist: float = 0.0  # float

    def pack(self):
        return struct.pack(
            EPUCK_NEIGHBOUR_PACKET_FMT_STR,
            self.robot_id,
            self.host.encode("ascii"),
            self.port,
            self.dist,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        args = struct.unpack(EPUCK_NEIGHBOUR_PACKET_FMT_STR, buffer)
        obj = cls(*args)
        obj.host = obj.host.decode("ascii").rstrip("\x00")  # type: ignore
        return obj

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_NEIGHBOUR_PACKET_FMT_STR)


EPUCK_KNOWLEDGE_PACKET_FMT_STR: str = ENDIAN_FMT + f"B{ROBOT_ID_TYPE_FMT_STR}B"
EPUCK_KNOWLEDGE_PACKET_ID: int = 0x22


@dataclass
class EpuckKnowledgePacket:
    known_ids: list[int]  # list of robot_id_type
    id: int = 0x22  # byte
    robot_id: int = 0  # see above
    N: int = 0x0  # byte

    def pack(self):
        return struct.pack(
            EPUCK_KNOWLEDGE_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.N,
        ) + struct.pack(ROBOT_ID_TYPE_FMT_STR * self.N, *self.known_ids)

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_KNOWLEDGE_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_KNOWLEDGE_PACKET_ID}"
            )

        start_len = struct.calcsize(EPUCK_KNOWLEDGE_PACKET_FMT_STR)
        id, robot_id, N = struct.unpack(
            EPUCK_KNOWLEDGE_PACKET_FMT_STR, buffer[:start_len]
        )
        known_id_buffer = buffer[
            start_len : struct.calcsize(ROBOT_ID_TYPE_FMT_STR) * N + start_len
        ]
        return cls(
            id=id,
            robot_id=robot_id,
            N=N,
            known_ids=list(struct.unpack(ROBOT_ID_TYPE_FMT_STR * N, known_id_buffer)),
        )

    @classmethod
    def calcsize(cls):
        return (
            struct.calcsize(EPUCK_KNOWLEDGE_PACKET_FMT_STR)
            + struct.calcsize(ROBOT_ID_TYPE_FMT_STR) * MAX_ROBOTS
        )


EPUCK_ADDRESS_KNOWLEDGE_PACKET_FMT_STR: str = (
    ENDIAN_FMT + f"B{ROBOT_ID_TYPE_FMT_STR}B{MAX_HOST_LEN+1}s"
)
EPUCK_ADDRESS_KNOWLEDGE_PACKET_ID: int = 0x22


@dataclass
class EpuckAddressKnowledgePacket:
    known_ids: list[int]  # list of byte
    id: int = 0x23  # byte
    robot_id: int = 0  # see above
    N: int = 0x0  # byte
    address: str = ""  # str

    def pack(self):
        return struct.pack(
            EPUCK_ADDRESS_KNOWLEDGE_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.N,
        ) + bytes(self.known_ids)

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_ADDRESS_KNOWLEDGE_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_ADDRESS_KNOWLEDGE_PACKET_ID}"
            )

        start_len = struct.calcsize(EPUCK_ADDRESS_KNOWLEDGE_PACKET_FMT_STR)
        id, robot_id, N, address = struct.unpack(
            EPUCK_ADDRESS_KNOWLEDGE_PACKET_FMT_STR, buffer[:start_len]
        )
        address = address.decode("ascii").rstrip("\x00")  # type: ignore
        return cls(
            id=id,
            robot_id=robot_id,
            N=N,
            address=address,
            known_ids=list(buffer[start_len : N + start_len]),
        )

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_ADDRESS_KNOWLEDGE_PACKET_FMT_STR) + MAX_ROBOTS
