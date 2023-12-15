#!/usr/bin/env python3

import rclpy
import csv
import os
import random

from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml

class DualSubscriber(Node):

    def __init__(self):
        super().__init__('dual_subscriber')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.folder_name = ""
        self.waypoints = []
        self.mission_params = {}
        self.sent = False
        self.current_iteration = 0

        self.subscription1 = self.create_subscription(
            String,
            '/gazebo/state',
            self.listener_callback1,
            qos)
        self.subscription2 = self.create_subscription(
            String,
            '/iteration/state',
            self.listener_callback2,
            qos)
        self.last_msg_topic1 = None
        self.last_msg_topic2 = None
        
        self.create_timer(5, self.timer_callback)

    def _init_mission_params(self) -> None:
        config_path = get_package_share_directory("px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

    def _generate_mission(self) -> None:
        self.waypoints = []

        for i in range(self.mission_params['iterations']):
            num_points = random.randint(self.mission_params['number_of_waypoints']['lower'],
                                        self.mission_params['number_of_waypoints']['upper'])
            self.waypoints.append(self._generate_random_path(num_points))

            if not os.path.exists(f"{self.folder_name}/iteration_{i}"):
                os.makedirs(f"{self.folder_name}/iteration_{i}")

        with open(f"{self.folder_name}/waypoints.csv", mode='w', newline='') as csvfile:
            wr = csv.writer(csvfile, quoting=csv.QUOTE_ALL, dialect="excel")
            wr.writerows(self.waypoints)
            self.get_logger().warning(f"Waypoints dumped to {self.folder_name}/waypoints.csv")

    def _generate_random_path(self, num_points) -> None:
        points = [[0.0, 0.0, -0.1, 0.0]] + self._generate_random_points(num_points)
        points = self._nearest_neighbour_path(points)
        points.append(points[0])
        return points

    def _generate_random_points(self, num_points) -> list:
        points = []

        ns_bounds = self.mission_params['boundaries']['north_south']
        ew_bounds = self.mission_params['boundaries']['east_west']
        altitude_bounds = self.mission_params['boundaries']['altitude']
        yaw_bounds = self.mission_params['boundaries']['yaw']

        for _ in range(num_points):
            x = random.uniform(ns_bounds['lower'], ns_bounds['upper'])
            y = random.uniform(ew_bounds['lower'], ew_bounds['upper'])
            altitude = random.uniform(altitude_bounds['lower'], altitude_bounds['upper'])
            yaw = random.uniform(yaw_bounds['lower'], yaw_bounds['upper'])
            points.append([x, y, altitude, yaw])

        return points
    
    def _nearest_neighbour_path(self, points) -> list:
        if not points:
            return []

        path = [points[0]]
        remaining_points = set(map(tuple, points[1:]))

        while remaining_points:
            last_point = path[-1]
            nearest = min(remaining_points, key=lambda p: (p[0] - last_point[0]) ** 2 + (p[1] - last_point[1]) ** 2)
            path.append(list(nearest))
            remaining_points.remove(nearest)

        return path
    
    def listener_callback1(self, msg):
        self.last_msg_topic1 = msg.data
        self.get_logger().info(f"I heard: {self.last_msg_topic1} from topic1")

    def listener_callback2(self, msg):
        self.last_msg_topic2 = msg.data
        self.get_logger().info(f"I heard: {self.last_msg_topic2} from topic2")

    def timer_callback(self):
        if self.last_msg_topic1 is None or self.last_msg_topic2 is None:
            return
        
        if not self.sent:
            self.sent = True
        elif self.last_msg_topic2 != "COMPLETED":
            self.get_logger().error(f"PATH COMPLETED")
        elif self.last_msg_topic1 != "ACTIVE":
            self.get_logger().error(f"SIM NOT RUNNING")
        else:
            self.sent = False
            self.get_logger().error(f"NEXT")
            

def main(args=None):
    rclpy.init(args=args)

    dual_subscriber = DualSubscriber()

    try:
        rclpy.spin(dual_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        dual_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
