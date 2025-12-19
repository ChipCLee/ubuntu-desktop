import logging
import struct
import time
from typing import Any, Dict

from franka_sim.franka_protocol import RobotMode

logger = logging.getLogger(__name__)


class GripperState:
    """Manages the state of the Franka robot simulation"""

    def __init__(self):
        """Initialize the robot state with default values"""
        self.state = self._initialize_state()

    def _initialize_state(self) -> Dict[str, Any]:
        """Initialize the robot state dictionary with default values"""
        return {
            "message_id": 0,
            "width": 0,  
            "max_width": 0,
            "is_grasped": False,
            "temperature": 0,
            "time": 0.0,
        }

    def pack_state(self) -> bytes:
        """Pack robot state into binary format for UDP transmission"""
        state = bytearray()

        # Pack state in libfranka-expected order
        state.extend(struct.pack("<Q", self.state["message_id"]))
        state.extend(struct.pack("<d", self.state["width"]))
        state.extend(struct.pack("<d", self.state["max_width"]))
        state.extend(struct.pack("<B", self.state["is_grasped"]))
        state.extend(struct.pack("<d", self.state["temperature"]))
        state.extend(struct.pack("<d", self.state["time"]))

        return bytes(state)

    def update(self):
        """Update the robot state for the next iteration"""
        self.state["message_id"] = int(time.time() * 1000)
        self.state["time"] = time.time()
        if self.state["message_id"] % 1000 == 0:  # Log every 1000 updates
            logger.debug(
                f"Gripper state updated: message_id={self.state['message_id']}, "
            )
