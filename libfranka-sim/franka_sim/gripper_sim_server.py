#!/usr/bin/env python3

import argparse
import enum
import logging
import select
import socket
import struct
import threading
import time
import platform  # Import platform module
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from franka_sim.franka_genesis_sim import ControlMode, FrankaGenesisSim
from franka_sim.gripper_protocol import (
    COMMAND_PORT,
    Command,
    ConnectStatus,
    MessageHeader,
    GraspCommand,
    MoveCommand,
    HomingCommand,
    MoveStatus,
)
from franka_sim.gripper_state import GripperState

# Configure detailed logging for debugging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(thread)d %(message)s")
logger = logging.getLogger(__name__)


class GripperMode(enum.IntEnum):
    """Operating modes of the Franka robot"""

    kOther = 0
    kIdle = 1
    kGrasp = 2


class GripperSimServer:
    """
    A simulation server implementing the Franka robot control interface protocol.
    Handles both TCP command communication and UDP state updates.
    """

    def __init__(self, host="0.0.0.0", port=COMMAND_PORT, enable_vis=False, genesis_sim=None):
        """
        Initialize the Franka simulation server.

        Args:
            host: IP address to bind to (default: all interfaces)
            port: TCP port for command interface
            enable_vis: Enable visualization of the Genesis simulator
            genesis_sim: Optional pre-configured Genesis simulator instance for testing
        """
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.transmitting_state = False
        self.library_version = 9  # Current libfranka version
        self.command_socket = None  # UDP socket for receiving commands
        self.current_motion_id = 0
        self.client_socket = None
        self.tcp_thread = None
        self.udp_socket = None
        self.client_address = None
        self.client_udp_port = None
        self.control_mode = ControlMode.NONE
        self.connection_running = False  # New flag for per-connection state

        # Initialize Genesis simulator
        if genesis_sim is None:
            logger.info("Initializing simulation")
            self.genesis_sim = FrankaGenesisSim(enable_vis=enable_vis)
            logger.info("Simulation initialized")
        else:
            self.genesis_sim = genesis_sim

        self.gripper_state = GripperState()

    def reset_state(self):
        """Reset all connection-specific state variables for a new connection"""
        self.transmitting_state = False
        self.current_motion_id = 0
        self.client_socket = None
        self.tcp_thread = None
        self.udp_socket = None
        self.client_address = None
        self.client_udp_port = None
        self.control_mode = ControlMode.NONE
        self.connection_running = False
        self.gripper_state = GripperState()  # Create fresh robot state for new connection

    def receive_exact(self, sock: socket.socket, size: int) -> Optional[bytes]:
        """
        Receive exactly 'size' bytes from the socket.

        Args:
            sock: Socket to receive from
            size: Number of bytes to receive

        Returns:
            bytes: Received data, or None if connection closed
        """
        data = bytearray()
        remaining = size

        while remaining > 0:
            try:
                logger.debug(f"Waiting to receive {remaining} bytes...")
                chunk = sock.recv(remaining)
                if not chunk:
                    logger.error("Connection closed while receiving data")
                    return None
                logger.debug(f"Received chunk of {len(chunk)} bytes")
                data.extend(chunk)
                remaining -= len(chunk)
            except socket.error as e:
                logger.error(f"Socket error while receiving: {e}")
                return None

        logger.debug(f"Successfully received all {size} bytes")
        return bytes(data)

    def receive_message(self, client_socket) -> Tuple[MessageHeader, Optional[bytes]]:
        """
        Receive a complete message following the libfranka protocol.

        Returns:
            Tuple of (MessageHeader, Optional[payload])
        """
        logger.debug("Attempting to receive message header (10 bytes)...")
        header_data = self.receive_exact(client_socket, 10)
        if not header_data:
            raise ConnectionError("Failed to receive message header")

        header = MessageHeader.from_bytes(header_data)
        logger.debug(
            f"[Gripper] Parsed header: command={Command(header.command).name}, "
            f"command_id={header.command_id}, size={header.size}"
        )

        payload_size = header.size - 10
        payload = None
        if payload_size > 0:
            logger.debug(f"Expecting payload of {payload_size} bytes")
            payload = self.receive_exact(client_socket, payload_size)
            if not payload:
                raise ConnectionError("Failed to receive message payload")
            logger.debug(f"Successfully received payload: {payload.hex()}")

        return header, payload

    def send_response(
        self, client_socket, command: int, command_id: int, status: ConnectStatus, version: int
    ):
        """
        Send a response message following the libfranka protocol.
        """
        # Total message size includes header (12 bytes) + response data (status + version + padding)
        total_size = 10 + 8  # 8 = 2(status) + 2(version) + 4(padding)

        # Construct and send header
        header = MessageHeader(command, command_id, 0, total_size)
        header_bytes = header.to_bytes()

        # Construct response data (status + version + 4 bytes padding)
        response_data = struct.pack("<HH4x", status.value, version)

        # Send complete message
        client_socket.sendall(header_bytes + response_data)
        logger.info(
            f"Sent response: command={Command(command).name}, "
            f"command_id={command_id}, status={status.name}"
        )

    def start_command_receiver(self):
        """Start UDP command receiver on specified port"""
        try:
            self.command_thread = threading.Thread(target=self._handle_commands)
            self.command_thread.daemon = True
            self.command_thread.start()

        except Exception as e:
            logger.error(f"Error starting command receiver: {e}", exc_info=True)

    def _handle_commands(self):
        """Handle incoming UDP robot commands"""
        logger.info("Command handler thread started")

        try:
            logger.info("Starting UDP command polling")
            # --- START CROSS-PLATFORM FIX ---
            timeout_s = 0.001  # 1ms timeout in seconds
            # Check if poll() is available and we are not on Windows
            use_poll = hasattr(select, "poll") and platform.system() != "Windows"

            if use_poll:
                # Use poll() on Linux/macOS
                poller = select.poll()
                poller.register(self.udp_socket, select.POLLIN)
                logger.debug(f"Using select.poll() for UDP commands")
                timeout_ms = int(timeout_s * 1000)
            else:
                # Use select.select() on Windows
                inputs = [self.udp_socket]
                logger.debug(f"Using select.select() for UDP commands (Windows compatible)")
            # --- END CROSS-PLATFORM FIX ---

            # --- START SHUTDOWN FIX ---
            while self.running and self.transmitting_state:
            # --- END SHUTDOWN FIX ---
                # --- START ROBUSTNESS FIX ---
                # Check if the socket is valid at the start of each loop
                if not self.running or not self.udp_socket:
                    logger.info("UDP socket is closed or server is stopping, exiting command handler.")
                    break
                # --- END ROBUSTNESS FIX ---

                # --- START CROSS-PLATFORM FIX ---
                try:
                    if use_poll:
                        events = poller.poll(timeout_ms)  # poll timeout is in ms
                        if not events:
                            continue
                        # We only registered one socket, so just check if there's any event
                        is_readable = len(events) > 0
                    else:
                        inputs = [self.udp_socket]  # Re-set inputs in case socket was None (though check above should catch it)
                        readable, _, _ = select.select(inputs, [], [], timeout_s)  # select timeout is in s
                        if not readable:
                            continue
                        is_readable = self.udp_socket in readable
                except Exception as e:
                    if self.running:  # Don't log errors if we're stopping
                        logger.error(f"Error during select/poll: {e}")
                    continue  # Continue loop

                if self.udp_socket is None:
                    # logger.debug("UDP socket is None, skipping command handling")
                    continue
                
                # logging.debug(f"gripper events = {readable}")
                if is_readable:
                    # --- END CROSS-PLATFORM FIX ---
                    expected_size = (
                        8 + (8 + 8 + 1 + 8 + 8)
                    )
                    command = None

                    try:
                        data, addr = self.udp_socket.recvfrom(expected_size)
                        if len(data) != expected_size:
                            logger.warning(
                                f"Got a UDP packet with wrong size! Expected {expected_size} \
                                bytes, got {len(data)} bytes"
                            )
                            continue
                        # Unpack the command data
                        offset = 0

                        # Unpack message_id
                        message_id = struct.unpack("<Q", data[offset : offset + 8])[0]
                        offset += 8

                        # Unpack MotionGeneratorCommand
                        width = struct.unpack("<d", data[offset : offset + 8])
                        offset += 8

                        max_width = struct.unpack("<d", data[offset : offset + 8])
                        offset += 8

                        is_grasped = struct.unpack("<B", data[offset : offset + 1])
                        offset += 1

                        temperature = struct.unpack("<d", data[offset : offset + 8])
                        offset += 8

                        time = struct.unpack("<d", data[offset : offset + 8])

                        command = {
                            "message_id": message_id,
                            "width": width,
                            "max_width": max_width,
                            "is_grasped": is_grasped,
                            "temperature": temperature,
                            "time": time,
                        }

                    except BlockingIOError:
                        continue # Not an error, just no data
                    except (socket.error, OSError) as e:
                        # If we are no longer transmitting, this error is expected
                        # as the socket is being closed.
                        if not self.transmitting_state:
                            logger.debug("Socket closed during shutdown, command handler exiting.")
                            break # Exit loop gracefully
                        
                        # If we are still supposed to be transmitting, then it's a real error
                        logger.error(f"Error receiving message: {e}")
                        break # Exit loop on unexpected error
                    except Exception as e:
                        # Catch any other unexpected error (like unpacking)
                        logger.error(f"Unexpected error in command handler: {e}", exc_info=True)
                        break
                

        except Exception as e:
            logger.error(f"Error in read_step: {e}")

    def handle_homing_command(self, client_socket, header: MessageHeader) -> None:
        """Handle Homing command received over TCP"""
        try:
            # Parse the homing command
            try:
                homing_cmd = HomingCommand.from_bytes(None)
            except ValueError as e:
                logger.error(f"Error handling Homing command1: {e}")
                self.send_homing_response(
                    client_socket,
                    command_id=header.command_id,
                    status=MoveStatus.kInvalidArgumentRejected,
                )
                return

            logger.info(f"Received Homing command: {homing_cmd}")
            self.genesis_sim.homming_gripper()

            self.current_motion_id = header.command_id

            # First send motion started response
            logger.info("Sending kMotionStarted response")
            self.send_homing_response(
                client_socket, command_id=header.command_id, status=MoveStatus.kSuccess
            )

        except Exception as e:
            logger.error(f"Error handling Homing command2: {e}")
            # Send error response
            self.send_homing_response(
                client_socket, command_id=header.command_id, status=MoveStatus.kAborted
            )
    
    def handle_stop_move_command(self, client_socket, header: MessageHeader):
        """Handle StopMove command received over TCP"""
        try:
            logger.info("Processing StopMove command")

            # Send success response for StopMove first
            total_size = 10 + 4  # Header (10) + status (1) + padding (3)
            response_header = MessageHeader(Command.kStopMove,  header.command_id, 0,total_size)
            header_bytes = response_header.to_bytes()

            # Status 0 = Success
            response_data = struct.pack("<B3x", 0)  # 1 byte status + 3 bytes padding

            client_socket.sendall(header_bytes + response_data)
            logger.info("Sent StopMove success response")

            # Send one final state with both modes set to idle
            if hasattr(self, "udp_socket") and self.udp_socket:

                # Send state with new message ID
                self.gripper_state.update()  # This increments message_id
                final_state = self.gripper_state.pack_state()
                self.udp_socket.sendto(final_state, (self.client_address, self.client_udp_port))
                logger.info(
                    f"Sent final gripper_state with message_id:\
                          {self.gripper_state.state['message_id']}"
                )

            # Stop robot state transmission
            self.transmitting_state = False
            logger.info("Stopped robot state transmission")

            # Send Move response to break the waiting loop in the client
            if self.current_motion_id:
                # Create a Move response header
                move_response_header = MessageHeader(Command.kGrasp,  self.current_motion_id, 0,14)
                move_header_bytes = move_response_header.to_bytes()
                move_response_data = struct.pack("<B3x", MoveStatus.kSuccess)
                client_socket.sendall(move_header_bytes + move_response_data)
                logger.info(f"Sent Move success response for motion ID: {self.current_motion_id}")
                self.current_motion_id = 0

            # Set connection_running to False instead of self.running
            self.connection_running = False

        except Exception as e:
            logger.error(f"Error handling StopMove command: {e}")
            # Send error response
            total_size = 10 + 4
            response_header = MessageHeader(Command.kStopMove,  header.command_id,0, total_size)
            header_bytes = response_header.to_bytes()
            response_data = struct.pack("<B3x", 5)  # Status 5 = Aborted
            client_socket.sendall(header_bytes + response_data)

    def handle_grasp_command(self, client_socket, header: MessageHeader, payload: bytes) -> None:
        """Handle Move command received over TCP"""
        try:
            # Parse the move command
            try:
                grasp_cmd = GraspCommand.from_bytes(payload)
            except ValueError as e:
                logger.error(f"Error handling Move command: {e}")
                self.send_grasp_response(
                    client_socket,
                    command_id=header.command_id,
                    status=MoveStatus.kInvalidArgumentRejected,
                )
                return

            logger.info(
                f"Received Grasp command: width={grasp_cmd.width}, "
                f"Received Grasp command= speed={grasp_cmd.speed}, "
            )

            self.genesis_sim.gripper_grasp(grasp_cmd)

            # First send motion started response
            logger.info("Sending kMotionStarted response")
            self.send_grasp_response(
                client_socket, command_id=header.command_id, status=MoveStatus.kSuccess
            )
            logger.info(f"Motion started with ID: {self.current_motion_id}")

        except Exception as e:
            logger.error(f"Error handling Move command: {e}")
            # Send error response
            self.send_grasp_response(
                client_socket, command_id=header.command_id, status=MoveStatus.kAborted
            )

    def handle_move_command(self, client_socket, header: MessageHeader, payload: bytes) -> None:
        """Handle Move command received over TCP"""
        try:
            # Parse the move command
            try:
                cmd = MoveCommand.from_bytes(payload)
            except ValueError as e:
                logger.error(f"Error handling Move command: {e}")
                self.send_move_response(
                    client_socket,
                    command_id=header.command_id,
                    status=MoveStatus.kInvalidArgumentRejected,
                )
                return

            logger.info(
                f"Received Move command: width={cmd.width}, "
                f"Received Move command= speed={cmd.speed}, "
            )

            self.genesis_sim.gripper_move(cmd)

            # First send motion started response
            logger.info("Sending Move response")
            self.send_move_response(
                client_socket, command_id=header.command_id, status=MoveStatus.kSuccess
            )
            logger.info(f"Motion started with ID: {self.current_motion_id}")

        except Exception as e:
            logger.error(f"Error handling Move command: {e}")
            # Send error response
            self.send_move_response(
                client_socket, command_id=header.command_id, status=MoveStatus.kAborted
            )

    def send_grasp_response(self, client_socket, command_id: int, status: MoveStatus):
        """Send response to Grasp command"""
        try:
            # Total message size includes header (12 bytes) + response data (status + padding)
            total_size = 10 + 4  # 4 = 1(status) + 3(padding)

            # Construct and send header
            header = MessageHeader(Command.kGrasp, command_id, 0, total_size)
            header_bytes = header.to_bytes()

            # Construct response data (status + 3 bytes padding)
            logger.debug(f"Sending Grasp response with status: {status.name} (value={status.value})")
            # Ensure we're using the enum value, not the enum itself
            status_value = status.value if isinstance(status, MoveStatus) else status
            response_data = struct.pack("<B3x", status_value)

            # Send complete message
            message = header_bytes + response_data
            logger.debug(f"Sending Grasp response message: {message.hex()}")
            client_socket.sendall(message)
            logger.info(f"Sent Grasp response: command_id={command_id}, status={status.name}")
        except Exception as e:
            logger.error(f"Error sending Grasp response: {e}", exc_info=True)

    def send_homing_response(self, client_socket, command_id: int, status: MoveStatus):
        """Send response to Homing command"""
        try:
            # Total message size includes header (12 bytes) + response data (status + padding)
            total_size = 10 + 4  # 4 = 1(status) + 3(padding)

            # Construct and send header
            header = MessageHeader(Command.kHomming,  command_id, 1, total_size)
            header_bytes = header.to_bytes()

            # Construct response data (status + 3 bytes padding)
            logger.debug(f"Sending Homing response with status: {status.name} (value={status.value})")
            # Ensure we're using the enum value, not the enum itself
            status_value = status.value if isinstance(status, MoveStatus) else status
            response_data = struct.pack("<B3x", status_value)

            # Send complete message
            message = header_bytes + response_data
            logger.debug(f"Sending Homing response message: {message.hex()}")
            client_socket.sendall(message)
            logger.info(f"Sent Homing response: command_id={command_id}, status={status.name}")
        except Exception as e:
            logger.error(f"Error sending Homing response: {e}", exc_info=True)
    
    def send_move_response(self, client_socket, command_id: int, status: MoveStatus):
        """Send response to Move command"""
        try:
            # Total message size includes header (12 bytes) + response data (status + padding)
            total_size = 10 + 4  # 4 = 1(status) + 3(padding)

            # Construct and send header
            header = MessageHeader(Command.kMove,  command_id, 0, total_size)
            header_bytes = header.to_bytes()

            # Construct response data (status + 3 bytes padding)
            logger.debug(f"Sending Move response with status: {status.name} (value={status.value})")
            # Ensure we're using the enum value, not the enum itself
            status_value = status.value if isinstance(status, MoveStatus) else status
            response_data = struct.pack("<B3x", status_value)

            # Send complete message
            message = header_bytes + response_data
            logger.debug(f"Sending Move response message: {message.hex()}")
            client_socket.sendall(message)
            logger.info(f"Sent Move response: command_id={command_id}, status={status.name}")
        except Exception as e:
            logger.error(f"Error sending Move response: {e}", exc_info=True)

    def handle_tcp_messages(self, client_socket):
        """Handle TCP messages in a separate thread"""
        logger.info(f"[Gripper]TCP message handler thread started, running: {self.running}")
        while self.running:  # Keep the TCP thread running even after client disconnects
            try:
                # Check if socket is still connected
                try:
                    client_socket.getpeername()
                except socket.error as e:
                    logger.error("Socket disconnected")
                    # Instead of breaking, reset state and continue
                    self.transmitting_state = False
                    self.connection_running = False
                    logger.info("Resetting state and waiting for new client...")
                    break  # Break only from the inner loop

                # Try to peek at incoming data
                readable, _, _ = select.select([client_socket], [], [], 0.1)
                if not readable:
                    continue

                logger.debug(f"Gripper Data available on socket, attempting to receive...")
                header, payload = self.receive_message(client_socket)
                logger.info(
                    f"Processing command: {Command(header.command).name} (ID: {header.command_id})"
                )

                if header.command == Command.kHomming:
                    self.handle_homing_command(client_socket, header)
                elif header.command == Command.kGrasp:
                    logger.debug(f"Move command payload size: {len(payload)} bytes")
                    logger.debug(f"Move command payload hex: {payload.hex()}")
                    self.handle_grasp_command(client_socket, header, payload)
                elif header.command == Command.kMove:
                    self.handle_move_command(client_socket, header, payload)
                elif header.command == Command.kStopMove:
                    logger.info("Received Stop command")
                    self.handle_stop_move_command(client_socket, header)
                else:
                    logger.warning(
                        f"Unhandled command in TCP thread: {Command(header.command).name}"
                    )
            except ConnectionError as e:
                logger.error(f"Connection error in TCP thread: {e}")
                # Instead of breaking, reset state and continue
                self.transmitting_state = False
                self.connection_running = False
                logger.info("Connection error: Resetting state and waiting for new client...")
                break  # Break only from the inner loop
            except Exception as e:
                logger.error(f"Error in TCP thread: {e}", exc_info=True)
                if not self.running:  # Only break if server is shutting down
                    break
                # For other errors, reset state and continue
                self.transmitting_state = False
                self.connection_running = False
                logger.info("Error occurred: Resetting state and waiting for new client...")
                break  # Break only from the inner loop

        logger.info("TCP message handler thread ending")

    def handle_client(self, client_socket):
        """
        Handle initial client connection and start message handlers
        """
        try:
            # Reset state for new connection
            self.reset_state()

            self.client_socket = client_socket
            self.connection_running = True
            logger.info("Waiting for initial connect command...")

            # Handle initial connect message
            header, payload = self.receive_message(client_socket)

            if header.command != Command.kConnect:
                logger.error(f"Expected connect command, got {Command(header.command).name}")
                return

            if not payload or len(payload) < 4:
                logger.error("Invalid connect payload: Version or UDP port not found")
                return

            # Log the full payload for debugging
            logger.info(f"Connect payload hex: {payload.hex()}")

            # The payload structure is:
            # - uint16_t version
            # - uint16_t udp_port (from network.udpPort())
            version, network_udp_port = struct.unpack("<HH", payload[:4])
            logger.info(f"Received version: {version}, network UDP port: {network_udp_port}")
            # Send successful connect response
            self.send_response(
                client_socket,
                command=header.command,
                command_id=header.command_id,
                status=ConnectStatus.kSuccess,
                version=self.library_version,
            )
            logger.info("Sent connect response")

            # Start TCP message handler thread
            self.tcp_thread = threading.Thread(
                target=self.handle_tcp_messages, args=(client_socket,)
            )
            self.tcp_thread.daemon = True
            self.tcp_thread.start()
            logger.info("Started TCP message handler thread")

            # Start UDP state transmission
            client_address = client_socket.getpeername()[0]
            logger.info(f"Starting UDP transmission to {client_address}:{network_udp_port}")
            self.start_robot_state_transmission(client_address, network_udp_port)

            # Keep the connection thread alive
            while self.connection_running and self.running:
                time.sleep(0.1)

            # Wait for TCP thread to finish
            if self.tcp_thread and self.tcp_thread.is_alive():
                self.tcp_thread.join(timeout=1.0)

        except Exception as e:
            logger.error(f"Error handling client: {e}", exc_info=True)
        finally:
            logger.info("Closing client connection")
            if client_socket:
                client_socket.close()
            # Clean up connection state
            self.reset_state()

            # Make sure UDP socket is closed
            if self.udp_socket:
                try:
                    self.udp_socket.close()
                except Exception as e:
                    logger.error(f"Error closing UDP socket: {e}")
                self.udp_socket = None

    def start_robot_state_transmission(self, client_address: str, client_udp_port: int):
        """
        Start UDP transmission of robot state updates.
        """
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # --- START BIND FIX ---
            # Bind the UDP socket to the same port as the TCP server
            # This is required for the client to know where to send UDP commands
            try:
                self.udp_socket.bind((self.host, self.port))
                logger.info(f"UDP socket bound to {self.host}:{self.port}")
            except Exception as e:
                logger.error(f"Failed to bind UDP socket: {e}")
                raise
            # --- END BIND FIX ---
            
            logging.info(f"Created UDP socket {self.udp_socket}")

            # TODO move to somewhere appropriate
            self.start_command_receiver()

            # port of the udp_socket
            udp_port = self.udp_socket.getsockname()[1]
            logger.debug(f"UDP port: {udp_port}")
            self.client_address = client_address
            self.client_udp_port = client_udp_port
            # Initialize timing statistics
            total_cycles = 0
            total_genesis_time = 0
            total_cycle_time = 0
            last_stats_time = time.time()

            logger.info(f"Starting UDP transmission to {client_address}:{client_udp_port}")
            self.transmitting_state = True
            first_state_sent = False

            while self.running and self.connection_running and self.transmitting_state:
                try:
                    cycle_start = time.time()

                    genesis_start = time.time()
                    sim_state = self.genesis_sim.get_gripper_state()
                    genesis_time = time.time() - genesis_start
                    total_genesis_time += genesis_time

                    # Initialize q_d to current q on first state update if not already set
                    # if not first_state_sent:
                    #     self.robot_state.state["q_d"] = sim_state["q"]

                    self.gripper_state.state.update(sim_state)

                    # Pack and send current robot state
                    state = self.gripper_state.pack_state()

                    if self.udp_socket and not self.udp_socket._closed:
                        self.udp_socket.sendto(state, (client_address, client_udp_port))

                    # After first state is sent, send a Move success response
                    if not first_state_sent and self.current_motion_id:
                        self.send_grasp_response(
                            self.client_socket,
                            command_id=self.current_motion_id,
                            status=MoveStatus.kSuccess,
                        )
                        first_state_sent = True

                    # Update state for next iteration
                    self.gripper_state.update()
                    # Calculate cycle statistics
                    cycle_time = time.time() - cycle_start
                    total_cycle_time += cycle_time
                    total_cycles += 1

                    # Log statistics every second
                    if time.time() - last_stats_time >= 1.0:
                        avg_genesis_time = (
                            total_genesis_time / total_cycles
                        ) * 1000  # Convert to ms
                        avg_cycle_time = (total_cycle_time / total_cycles) * 1000  # Convert to ms
                        freq = total_cycles / (time.time() - last_stats_time)

                        logger.info(
                            f"State Update Stats - Freq: {freq:.1f}Hz, "
                            f"Genesis Time: {avg_genesis_time:.2f}ms, "
                            f"Total Cycle: {avg_cycle_time:.2f}ms"
                        )

                        # Reset statistics
                        total_cycles = 0
                        total_genesis_time = 0
                        total_cycle_time = 0
                        last_stats_time = time.time()

                except Exception as e:
                    logger.error(f"Error in UDP transmission: {e}")
                    if not self.running or not self.connection_running:
                        break

        except Exception as e:
            logger.error(f"Error in robot state transmission: {e}")
        finally:
            self.transmitting_state = False
            if self.udp_socket:
                try:
                    self.udp_socket.close()
                except Exception as e:
                    logger.error(f"Error closing UDP socket: {e}")
                self.udp_socket = None

    def run_server(self):
        """Main server loop that runs in a separate thread when visualization is enabled"""
        try:
            logger.info("Starting TCP server initialization...")
            # Start TCP server
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            logger.info("Created server socket")

            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Only set SO_REUSEPORT if available, handle windows platform
            if hasattr(socket, "SO_REUSEPORT"):
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.server_socket.settimeout(1.0)
            logger.info("Set socket options")

            try:
                logger.info(f"Attempting to bind to {self.host}:{self.port}")
                self.server_socket.bind((self.host, self.port))
                logger.info("Successfully bound to address")
            except OSError as e:
                if e.errno == 48:  # Address already in use
                    logger.warning(
                        f"Port {self.port} is in use, attempting to force close and rebind..."
                    )
                    self.server_socket.close()
                    time.sleep(1)
                    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    if hasattr(socket, "SO_REUSEPORT"): # Add check here too
                        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                    self.server_socket.bind((self.host, self.port))
                    logger.info("Successfully rebound to address after force close")
                else:
                    logger.error(f"Failed to bind: {e}")
                    raise

            self.server_socket.listen(1)
            logger.info(f"Server listening on {self.host}:{self.port}")
            self.running = True

            while self.running:
                try:
                    # Reset state before accepting new connection
                    self.reset_state()
                    logger.info("Server ready for new client connection...")

                    client_socket, address = self.server_socket.accept()
                    client_ip = address[0]
                    client_port = address[1]
                    logger.info(f"New connection from {client_ip}:{client_port}")

                    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                    # Handle client - this will block until client disconnects
                    self.handle_client(client_socket)

                    logger.info("Client session ended, ready for next client")

                except socket.timeout:
                    # Just continue waiting for new connections
                    continue
                except Exception as e:
                    logger.error(f"Connection handling error: {e}", exc_info=True)
                    if "client_socket" in locals():
                        try:
                            client_socket.close()
                        except Exception as e:
                            logger.error(f"Error closing client socket: {e}")
                    # Reset state and continue listening for next client
                    self.reset_state()
                    continue

        except Exception as e:
            logger.error(f"Server start error: {e}", exc_info=True)
            self.running = False
        finally:
            self.cleanup()

    def start(self):
        """Start the TCP server and Genesis simulator"""
        try:
            self.running = True
            logger.info("Starting server and simulation")

            # Initialize Genesis simulator first
            # self.genesis_sim.initialize_simulation()
            logger.info("Genesis simulation initialized")

            if self.genesis_sim.enable_vis:
                # Run server in a background thread when visualization is enabled
                server_thread = threading.Thread(target=self.run_server)
                server_thread.daemon = True
                server_thread.start()
                logger.info("Gripper Server running in background thread")

                # Start Genesis simulator (visualization) in main thread
                # logger.info("Starting Genesis simulator with visualization")
                # self.genesis_sim.start()
            else:
                # Without visualization, run server in main thread
                logger.info("Starting TCP/UDP server")
                self.run_server()
                # Start Genesis simulator without visualization
                # self.genesis_sim.start()

        except Exception as e:
            logger.error(f"Server start error: {e}", exc_info=True)
            self.cleanup()
            raise

    def cleanup(self):
        """Clean up all resources"""
        logger.info("Cleaning up server resources...")

        # Stop all running operations
        self.running = False
        self.transmitting_state = False
        self.connection_running = False

        # Clean up client socket
        if hasattr(self, "client_socket") and self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
            except socket.error:
                pass
            try:
                self.client_socket.close()
            except socket.error:
                pass
            self.client_socket = None

        # Clean up server socket
        if hasattr(self, "server_socket") and self.server_socket:
            try:
                self.server_socket.shutdown(socket.SHUT_RDWR)
            except socket.error:
                pass
            try:
                self.server_socket.close()
            except socket.error:
                pass
            self.server_socket = None

        # Clean up command socket
        if hasattr(self, "command_socket") and self.command_socket:
            try:
                self.command_socket.shutdown(socket.SHUT_RDWR)
            except socket.error:
                pass
            try:
                self.command_socket.close()
            except socket.error:
                pass
            self.command_socket = None

        # Clean up UDP socket
        if hasattr(self, "udp_socket") and self.udp_socket:
            try:
                self.udp_socket.close()
            except socket.error:
                pass
            self.udp_socket = None

        # Wait for any remaining operations to complete
        time.sleep(0.1)

        # Reset all state
        self.reset_state()
        self.running = False

    def stop(self):
        """Stop the server and clean up resources"""
        logger.info("Stopping server...")
        self.running = False
        self.connection_running = False
        self.transmitting_state = False
        self.cleanup()
        # Stop Genesis simulator
        self.genesis_sim.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-v",
        "--vis",
        action="store_true",
        default=False,
        help="Enable visualization of the Genesis simulator",
    )
    args = parser.parse_args()

    server = GripperSimServer(enable_vis=args.vis)
    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Shutting down server...")
        server.stop()


if __name__ == "__main__":
    main()