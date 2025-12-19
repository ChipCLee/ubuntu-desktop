from franka_sim.gripper_protocol import *

class TestProtocolParsing(unittest.TestCase):
    def test_message_header(self):
        data = struct.pack("<HHHI", Command.kGrasp, 123, 0, 42)
        header = MessageHeader.from_bytes(data)
        self.assertEqual(header.command, Command.kGrasp)
        self.assertEqual(header.command_id, 123)
        self.assertEqual(header.ret, 0)
        self.assertEqual(header.size, 42)

    def test_grasp_command(self):
        data = struct.pack("<5d", 0.08, 0.1, 20.0, 0.005, 0.005)
        cmd = GraspCommand.from_bytes(data)
        self.assertAlmostEqual(cmd.width, 0.08)
        self.assertAlmostEqual(cmd.speed, 0.1)
        self.assertAlmostEqual(cmd.force, 20.0)
        self.assertAlmostEqual(cmd.epsilon_inner, 0.005)
        self.assertAlmostEqual(cmd.epsilon_outer, 0.005)

    def test_move_command(self):
        data = struct.pack("<2d", 0.1, 0.2)
        cmd = MoveCommand.from_bytes(data)
        self.assertAlmostEqual(cmd.width, 0.1)
        self.assertAlmostEqual(cmd.speed, 0.2)

import unittest
from unittest.mock import MagicMock, patch
from franka_sim.gripper_sim_server import GripperSimServer

class TestGripperCommandHandling(unittest.TestCase):
    def setUp(self):
        self.mock_sim = MagicMock()
        self.server = GripperSimServer(genesis_sim=self.mock_sim)
        self.server.running = True
        self.server.connection_running = True

    @patch('socket.socket')
    def test_connect_command(self, mock_socket):
        mock_client = MagicMock()
        
        header_data = struct.pack("<HHHI", Command.kConnect, 1, 0, 18)
        payload = struct.pack("<HH", 9, 12345)  
        
        mock_client.recv.side_effect = [header_data[:10], payload]
        
        self.server.handle_client(mock_client)
        
        expected_header = struct.pack("<HHHI", Command.kConnect, 1, 0, 18)
        expected_data = struct.pack("<HH4x", ConnectStatus.kSuccess.value, 9)
        mock_client.sendall.assert_called_with(expected_header + expected_data)
