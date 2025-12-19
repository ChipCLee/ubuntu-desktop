#!/usr/bin/env python3
import argparse
import logging

from franka_sim import FrankaSimServer, GripperSimServer
import threading

# Configure logging to silence Numba debug output
logging.getLogger("numba").setLevel(logging.DEBUG)


def main():
    """Run the Franka simulation server."""
    # get command line arguments visualization
    parser = argparse.ArgumentParser(description="Run a Franka simulation server")
    parser.add_argument(
        "-v",
        "--vis",
        action="store_true",
        default=False,
        help="Enable visualization of the Genesis simulator",
    )
    args = parser.parse_args()

    print(f"Starting Franka Simulation Server {'with' if args.vis else 'without'} visualization")
    print("Connect to the server using 'localhost' or '127.0.0.1' as the robot IP address")
    print("Press Ctrl+C to stop the server")

    server = FrankaSimServer(enable_vis=args.vis)

    logging.info("Starting Gripper simulation server...")
    gripper_server = GripperSimServer(enable_vis=args.vis, genesis_sim=server.genesis_sim)
    try:
        logging.info("--------------")
        arm_thread = threading.Thread(target=server.start)
        gripper_thread = threading.Thread(target=gripper_server.start)

        arm_thread.start()
        logging.info("arm server started in a separate thread")
        gripper_thread.start()
        logging.info("gripper server started in a separate thread")

        arm_thread.join()
        gripper_thread.join()

    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.stop()


if __name__ == "__main__":
    main()
