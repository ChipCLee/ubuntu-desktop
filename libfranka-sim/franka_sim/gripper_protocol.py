import enum
import struct
from dataclasses import dataclass
import logging

# Standard command port for Franka robot interface
COMMAND_PORT = 1338

class Command(enum.IntEnum):
    """Commands supported by the Franka robot interface protocol"""

    kConnect = 0
    kHomming = 1
    kGrasp = 2
    kMove = 3
    kStopMove = 4



class ConnectStatus(enum.IntEnum):
    """Connection status codes for the Franka protocol"""

    kSuccess = 0
    kIncompatibleLibraryVersion = 1


class MoveStatus(enum.IntEnum):
    """Status codes for Move command"""

    kSuccess = 0
    kMotionStarted = 1
    kPreempted = 2
    kPreemptedDueToActivatedSafetyFunctions = 3
    kCommandRejectedDueToActivatedSafetyFunctions = 4
    kCommandNotPossibleRejected = 5
    kStartAtSingularPoseRejected = 6
    kInvalidArgumentRejected = 7
    kReflexAborted = 8
    kEmergencyAborted = 9
    kInputErrorAborted = 10
    kAborted = 11

class RobotMode(enum.IntEnum):
    """Operating modes of the Franka robot"""

    kOther = 0
    kIdle = 1
    kMove = 2
    kGuiding = 3
    kReflex = 4
    kUserStopped = 5
    kAutomaticErrorRecovery = 6

# TODO: check all these usages!!!
@dataclass
class MessageHeader:
    """
    Represents the message header structure from libfranka.
    All messages begin with this 12-byte header.
    """
    command: Command  # Command type (uint32)
    command_id: int  # Unique command identifier (uint32)
    ret: int  # TODO: what's this
    size: int  # Total message size including header (uint32)
    @classmethod
    def from_bytes(cls, data: bytes) -> "MessageHeader":
        """Parse header from binary data using little-endian format"""
        command, command_id, ret,  size = struct.unpack("<HHHI", data)
        logging.info(f"command: {command}, command_id: {command_id},ret:{ret},  size: {size}")
        return cls(Command(command), command_id, ret, size)

    def to_bytes(self) -> bytes:
        """Convert header to binary format using little-endian"""
        return struct.pack("<HHHI",  self.command, self.command_id,self.ret, self.size)


@dataclass
class GraspCommand:
    """Represents a Move command request"""
    width: float  # Grasp width in meters
    speed: float  # Grasp speed in m/s
    force: float  # Grasp force in N
    epsilon_inner: float  # Inner epsilon in meters
    epsilon_outer: float  # Outer epsilon in meters
    @classmethod
    def from_bytes(cls, data: bytes) -> "GraspCommand":
        """Parse GraspCommand command from binary data"""
        
        width, speed, force, epsilon_inner, epsilon_outer = struct.unpack(
            "<5d", data
        )
        return cls(width, speed, force, epsilon_inner, epsilon_outer)
    
@dataclass
class MoveCommand:
    """Represents a Move command request"""
    width: float  # Grasp width in meters
    speed: float  # Grasp speed in m/s
    @classmethod
    def from_bytes(cls, data: bytes) -> "MoveCommand":
        """Parse GraspCommand command from binary data"""
        
        width, speed = struct.unpack(
            "<2d", data
        )
        return cls(width, speed)

@dataclass
class HomingCommand:
    @classmethod
    def from_bytes(cls, data: bytes) -> "HomingCommand":
        return cls()
