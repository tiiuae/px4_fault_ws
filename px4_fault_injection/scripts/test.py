#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String, Empty
import subprocess
import time
import signal
import os
import threading
import psutil

class GazeboControlNode(Node):
    def __init__(self):
        super().__init__('gazebo_control_node')
        self.process = None
        self.simulation_thread = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.publisher = self.create_publisher(String, '/gazebo/state', qos)
        self.activate_pub = self.create_publisher(Empty, "/drone_controller/activate", 10)
        self.deactivate_pub = self.create_publisher(Empty, "/drone_controller/deactivate", 10)
        self.initialise_pub = self.create_publisher(Empty, "/drone_controller/initialise", 10)

        self.subscription = self.create_subscription(
            String,
            '/gazebo/trigger',
            self.listener_callback,
            10)
        
        self.px4_root_dir = '/home/juniorsundar-unikie/Documents/new/PX4-Autopilot/'
        self.publish_message("IDLE")
        self.start_gazebo_simulation_thread()
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

    def start_gazebo_simulation_thread(self):
        if self.simulation_thread is not None:
            self.simulation_thread.join()  # Ensure previous thread is finished
        self.simulation_thread = threading.Thread(target=self.start_gazebo_simulation)
        self.simulation_thread.start()

    def start_gazebo_simulation(self):
        os.environ['HEADLESS'] = '1'
        os.chdir(self.px4_root_dir)
        command = ["make", "px4_sitl", "gz_x500"]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        for line in self.process.stdout:
            print(line, end='')

    def kill_px4_server(self):
        kill_command = ["killall", "px4"]
        subprocess.run(kill_command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(5)
        kill_command = ["killall", "ninja"]
        subprocess.run(kill_command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(5)
        kill_command = ["killall", "make"]
        subprocess.run(kill_command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # parent = psutil.Process(self.process.pid)
        # children = parent.children(recursive=True)
        # for child in children:
        #     child.kill()
        # parent.kill()
        # self.process = None
        # self.simulation_thread = None

    def listener_callback(self, msg):
        print("Received message:", msg.data)
        if msg.data == 'KILL_RE':
            self.publish_message("KILLED")
            self.kill_px4_server()
            time.sleep(10)
            self.publish_message("IDLE")
            self.start_gazebo_simulation_thread()
            time.sleep(10)
            self.prepare_drone()
            self.publish_message("ACTIVE")
        elif msg.data == 'KILL':
            self.publish_message("KILLED")
            self.kill_px4_server()
            time.sleep(10)
            self.publish_message("IDLE")
        elif msg.data == 'START':
            self.start_gazebo_simulation_thread()
            time.sleep(10)
            self.prepare_drone()
            self.publish_message("ACTIVE")

def main(args=None) -> None:
    print('Starting simulation_manager node...')
    rclpy.init(args=args)
    try:
        gazebo_control_node = GazeboControlNode()
        rclpy.spin(gazebo_control_node)
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass


if __name__ == '__main__':
    main()