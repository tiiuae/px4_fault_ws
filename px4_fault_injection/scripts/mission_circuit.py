#!/usr/bin/env python3

import asyncio
import csv
import os
import random
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from px4_msgs.msg import *
from std_msgs.msg import Int8, Empty, Float32MultiArray, String
from io import StringIO


class MissionCircuit(Node):

    SLEEP_TIME = 5

    def __init__(self) -> None:
        super().__init__('mission_circuit')

        self.folder_name = ""
        self.waypoints = []
        self.mission_params = {}

        self._create_directory()
        self._init_mission_params()
        self._create_pub_sub()
        self._generate_mission()
        self._run()

    def _create_pub_sub(self) -> None:
        self.get_clock().sleep_for(Duration(seconds=self.SLEEP_TIME*2))
        self.record_pub = self.create_publisher(Int8, "/mission_circuit/record",10)
        self.activate_pub = self.create_publisher(Empty, "/drone_controller/activate", 10)
        self.deactivate_pub = self.create_publisher(Empty, "/drone_controller/deactivate", 10)
        self.move_pub = self.create_publisher(Float32MultiArray, "/drone_controller/move_drone_NEDY", 10)
        self.param_float_pub = self.create_publisher(String, "/drone_controller/set_param_float", 10)
        self.param_int_pub = self.create_publisher(String, "/drone_controller/set_param_int", 10)
        return

    def _start_record(self, record_id):
        self.record_pub.publish(Int8(data=record_id))
        return

    def _end_record(self, record_id):
        self.record_pub.publish(Int8(data=record_id))
        return

    def _create_directory(self) -> None:
        self.folder_name = f"{os.getcwd()}/records/{str(int(self.get_clock().now().seconds_nanoseconds()[0]/100))}"
        if not os.path.exists(self.folder_name):
            os.makedirs(self.folder_name)
            self.get_logger().warning(f"Directory '{self.folder_name}' created successfully.")
        else:
            self.get_logger().info(f"Directory '{self.folder_name}' already exists.")

    def _csv_string_to_list(self, csv_string):
        csv_file = StringIO(csv_string)

        csv_reader = csv.reader(csv_file)
        row_list = next(csv_reader, [])

        return row_list

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

    def _init_mission_params(self) -> None:
        config_path = get_package_share_directory("px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

    def _generate_random_path(self, num_points) -> None:
        points = [[0.0,0.0,-0.1,0.0]] + self._generate_random_points(num_points)
        points = self._nearest_neighbour_path(points)
        points.append(points[0])
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
    
    def _run(self) -> None:

        self.activate_pub.publish(Empty())
        self.get_clock().sleep_for(Duration(seconds=self.SLEEP_TIME))

        for i in range(len(self.waypoints)):
            self._start_record(i)
            self.get_clock().sleep_for(Duration(seconds=self.SLEEP_TIME))
            for point in self.waypoints[i]:
                pos = Float32MultiArray()
                pos.data = point
                self.move_pub.publish(pos)
                self.get_clock().sleep_for(Duration(seconds=self.SLEEP_TIME))

            self._end_record(-1)

        self.get_logger().info("Stopping offboard")
        self.deactivate_pub.publish(Empty())
        return

def main(args=None) -> None:
    print('Starting mission_circuit node...')
    rclpy.init(args=args)
    try:
        mission_circuit = MissionCircuit()
        rclpy.spin(mission_circuit)
    except KeyboardInterrupt:
        pass
    
    try:
        rclpy.shutdown()
    except Exception as e:
        pass


if __name__ == '__main__':
    main()