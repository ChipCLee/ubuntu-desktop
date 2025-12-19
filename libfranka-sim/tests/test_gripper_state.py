import unittest
import struct
from franka_sim.gripper_state import GripperState

class TestGripperState(unittest.TestCase):
    def test_initial_state(self):
        state = GripperState()
        self.assertEqual(state.state["width"], 0)
        self.assertEqual(state.state["max_width"], 0)
        self.assertFalse(state.state["is_grasped"])
        self.assertEqual(state.state["temperature"], 0)
        self.assertEqual(state.state["time"], 0.0)

    def test_pack_state(self):
        state = GripperState()
        state.state.update({
            "message_id": 123456789,
            "width": 0.08,
            "max_width": 0.12,
            "is_grasped": True,
            "temperature": 45.0,
            "time": 1625097600.0
        })
        
        packed = state.pack_state()
        
        fmt = "<QddBdd"
        expected_size = struct.calcsize(fmt)
        self.assertEqual(len(packed), expected_size)
        
        unpacked = struct.unpack(fmt, packed)
        self.assertEqual(unpacked[0], 123456789)    # message_id
        self.assertAlmostEqual(unpacked[1], 0.08)   # width
        self.assertAlmostEqual(unpacked[2], 0.12)   # max_width
        self.assertEqual(unpacked[3], 1)            # is_grasped
        self.assertAlmostEqual(unpacked[4], 45.0)   # temperature
        self.assertAlmostEqual(unpacked[5], 1625097600.0)  # time
