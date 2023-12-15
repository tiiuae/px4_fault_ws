#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Empty
import time
import subprocess
import os


class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.publisher = self.create_publisher(String, '/gazebo/state', qos)
        self.activate_pub = self.create_publisher(Empty, "/drone_controller/activate", 10)
        self.deactivate_pub = self.create_publisher(Empty, "/drone_controller/deactivate", 10)
        self.initialise_pub = self.create_publisher(Empty, "/drone_controller/initialise", 10)

        self.subscription = self.create_subscription(String, '/gazebo/trigger', self.listener_callback2, 10)

        self.root_dir = os.getcwd()
        self.start_session_sh = f'{self.root_dir}/create_session.sh'
        self.kill_session_sh = f'{self.root_dir}/kill_session.sh'

        self.publish_message("IDLE")
        subprocess.call([self.start_session_sh])
        time.sleep(10)
        self.prepare_drone()
        self.publish_message("ACTIVE")

    def publish_message(self, data: str):
        self.publisher.publish(String(data=data))

    def prepare_drone(self):
        self.initialise_pub.publish(Empty())
        time.sleep(10)
        self.activate_pub.publish(Empty())
        time.sleep(5)

    def listener_callback2(self, msg):
        self.get_logger().info("Received message:", msg.data)
        if msg.data == 'KILL_RE':
            self.publish_message("KILLED")
            subprocess.call([self.kill_session_sh])
            time.sleep(10)
            self.publish_message("IDLE")
            subprocess.call([self.start_session_sh])
            time.sleep(10)
            self.prepare_drone()
            self.publish_message("ACTIVE")
        elif msg.data == 'KILL':
            self.publish_message("KILLED")
            subprocess.call([self.kill_session_sh])
            time.sleep(10)
            self.publish_message("IDLE")
        elif msg.data == 'START':
            subprocess.call([self.start_session_sh])
            time.sleep(10)
            self.prepare_drone()
            self.publish_message("ACTIVE")


def main(args=None):
    print('Starting simulation_manager node...')
    rclpy.init(args=args)
    try:
        simulation_manager = SimulationManager()
        rclpy.spin(simulation_manager)
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass


if __name__ == '__main__':
    main()
