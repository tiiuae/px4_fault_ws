#!/usr/bin/env python3

import rclpy
import csv
import os
import random
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int8
from ament_index_python.packages import get_package_share_directory
import yaml


class MissionManager(Node):
    """
    MissionManager Node for managing drone missions in a ROS2 environment.
    Handles the generation, storage, and execution of waypoints for drone navigation.
    """

    def __init__(self):
        super().__init__('mission_manager')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Initialization of member variables
        self.folder_name = ""
        self.waypoints = []
        self.mission_params = {}
        self.current_iteration = 0
        self.state = 1
        self.simulation_msg = None
        self.iteration_msg = 'COMPLETED'

        # ROS2 Subscribers and Publisher
        self.sim_subscriber = self.create_subscription(String, '/gazebo/state', self.sim_state_callback, qos)
        self.iteration_subscriber = self.create_subscription(String, '/iteration/state', self.iteration_callback, qos)
        self.iteration_pub = self.create_publisher(Float32MultiArray, '/iteration/waypoints', 1)
        self.iter_update = self.create_publisher(Int8, '/iteration/current', 1)

        # Mission setup
        self._create_directory()
        self._init_mission_params()
        self._generate_mission()

        # ROS2 Timer
        self.create_timer(5, self.timer_callback)

    def update(self, iteration_msg=None, simulation_msg=None) -> None:
        """
        Update the current iteration and simulation message.
        """
        if iteration_msg is not None:
            self.iteration_msg = iteration_msg
        if simulation_msg is not None:
            self.simulation_msg = simulation_msg
        self.process_state()

    def process_state(self):
        """
        Process the current state of the mission and transition to the next state as needed.
        """
        state_actions = {
            1: self.state_one,
            2: self.state_two,
            3: self.state_three,
            4: self.state_four
        }
        state_actions.get(self.state, lambda: None)()

    def state_one(self):
        """
        State 1: Publish waypoints and transition to the next state.
        """
        self.get_logger().debug("Entering State 1")
        self.msg_sent = False
        self.state = 2

        if self.current_iteration >= len(self.waypoints):
            self.state = 4
            return

        output = Float32MultiArray()
        output.data = np.array(self.waypoints[self.current_iteration]).reshape(1, -1)[0].tolist()
        self.iteration_pub.publish(output)
        self.iter_update.publish(Int8(data = self.current_iteration))
        self.current_iteration += 1

    def state_two(self):
        """
        State 2: Check the iteration message and transition accordingly.
        """
        self.get_logger().debug("In State 2")
        if self.iteration_msg == "COMPLETED":
            self.state = 1
            self.msg_sent = True
        elif self.iteration_msg == "PREEMPTED":
            self.state = 3

    def state_three(self):
        """
        State 3: Handle the preempted state based on simulation message.
        """
        self.get_logger().debug("In State 3")
        if self.simulation_msg == "ACTIVE":
            self.state = 1
            self.msg_sent = True

    def state_four(self):
        """
        State 4: Final state, indicating completion.
        """
        self.get_logger().debug("In State 4")
        # No further action required

    def _create_directory(self) -> None:
        """
        Create a directory to store mission records.
        """
        self.folder_name = f"{os.getcwd()}/records/{str(int(self.get_clock().now().seconds_nanoseconds()[0]/100))}"
        if not os.path.exists(self.folder_name):
            os.makedirs(self.folder_name)
            self.get_logger().warning(f"Directory '{self.folder_name}' created successfully.")
        else:
            self.get_logger().info(f"Directory '{self.folder_name}' already exists.")

    def _init_mission_params(self) -> None:
        """
        Initialize mission parameters from a configuration file.
        """
        config_path = get_package_share_directory("px4_fault_injection") + "/config/circuit_params.yaml"

        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

    def _generate_mission(self) -> None:
        """
        Generate waypoints for the mission.
        """
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

    def _generate_random_path(self, num_points) -> list:
        """
        Generate a random path with a specified number of waypoints.
        """
        points = [[0.0, 0.0, -0.1, 0.0]] + self._generate_random_points(num_points)
        points = self._nearest_neighbour_path(points)
        points.append(points[0])
        return points

    def _generate_random_points(self, num_points) -> list:
        """
        Generate random points for the path.
        """
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
        """
        Generate a path using the nearest neighbour algorithm.
        """
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

    def sim_state_callback(self, msg):
        """
        Callback for simulation state updates.
        """
        self.simulation_msg = msg.data
        self.get_logger().info(f"Simulation state is now: {self.simulation_msg}")

    def iteration_callback(self, msg):
        """
        Callback for iteration state updates.
        """
        self.iteration_msg = msg.data
        self.get_logger().info(f"Current iteration state is now: {self.iteration_msg}")

    def timer_callback(self):
        """
        Timer callback to periodically update the mission state.
        """
        if self.simulation_msg != 'ACTIVE' or self.iteration_msg is None:
            return

        self.update(iteration_msg=self.iteration_msg, simulation_msg=self.simulation_msg)


def main(args=None):
    print('Starting mission_manager node...')
    rclpy.init(args=args)

    mission_manager = MissionManager()

    try:
        rclpy.spin(mission_manager)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
