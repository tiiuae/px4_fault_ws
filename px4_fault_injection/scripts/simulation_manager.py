#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Empty
import time
import subprocess
import os


class SimulationManager(Node):
    """
    A ROS2 node that manages simulation states and interacts with external shell scripts
    to control a Gazebo simulation environment for a drone.
    """

    def __init__(self):
        """
        Initializes the SimulationManager node and sets up publishers, subscribers,
        and initial states.
        """
        super().__init__('simulation_manager')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers to communicate simulation state and control drone activation
        self.publisher = self.create_publisher(String, '/gazebo/state', qos)
        self.activate_pub = self.create_publisher(Empty, "/drone_controller/activate", 10)
        self.deactivate_pub = self.create_publisher(Empty, "/drone_controller/deactivate", 10)
        self.initialise_pub = self.create_publisher(Empty, "/drone_controller/initialise", 10)

        # Subscriber to listen for simulation control commands
        self.subscription = self.create_subscription(
            String, '/gazebo/trigger', self.listener_callback, 10)

        # Paths to shell scripts for starting and killing simulation sessions
        self.root_dir = os.getcwd()
        self.start_session_sh = os.path.join(self.root_dir, 'create_session.sh')
        self.kill_session_sh = os.path.join(self.root_dir, 'kill_session.sh')

        # Initialize simulation and drone states
        self.publish_message("IDLE")
        subprocess.call([self.start_session_sh])
        time.sleep(10)
        self.prepare_drone()
        self.publish_message("ACTIVE")

    def publish_message(self, data: str):
        """
        Publishes a string message to the '/gazebo/state' topic.

        Parameters:
            data (str): The message to be published.
        """
        self.publisher.publish(String(data=data))

    def prepare_drone(self):
        """
        Sends commands to initialize and activate the drone.
        """
        self.initialise_pub.publish(Empty())
        time.sleep(10)
        self.activate_pub.publish(Empty())
        time.sleep(5)

    def listener_callback(self, msg):
        """
        Handles incoming messages for simulation control.

        Parameters:
            msg (std_msgs.msg.String): The received message.
        """
        self.get_logger().info(f"Received message: {msg.data}")
        if msg.data == 'KILL_RE':
            self.restart_simulation()
        elif msg.data == 'KILL':
            self.kill_simulation()
        elif msg.data == 'START':
            self.start_simulation()

    def restart_simulation(self):
        """
        Restarts the simulation session.
        """
        self.kill_simulation()
        self.start_simulation()

    def kill_simulation(self):
        """
        Kills the current simulation session.
        """
        self.publish_message("KILLED")
        subprocess.call([self.kill_session_sh])
        time.sleep(10)
        self.publish_message("IDLE")

    def start_simulation(self):
        """
        Starts a new simulation session.
        """
        subprocess.call([self.start_session_sh])
        time.sleep(10)
        self.prepare_drone()
        self.publish_message("ACTIVE")


def main(args=None):
    print('Starting simulation_manager node...')
    rclpy.init(args=args)
    simulation_manager = SimulationManager()
    try:
        rclpy.spin(simulation_manager)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
