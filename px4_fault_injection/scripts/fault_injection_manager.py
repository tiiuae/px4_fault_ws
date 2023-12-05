#!/usr/bin/env python3

import csv
import os

import rclpy
from rclpy.node import Node
import random

from std_msgs.msg import Int8
from std_srvs.srv import SetBool
from io import StringIO
from ament_index_python.packages import get_package_share_directory
import yaml


class FaultInjectionManager(Node):

    SLEEP_TIME = 5

    def __init__(self) -> None:
        super().__init__('fault_injection_manager')

        self.folder_name = ""
        self.mission_params = {}
        self.iter_active = False
        self.faulty_runs = False
        self.fault_active = False

        self._init_mission_params()
        self._create_directory()

        self.cli = self.create_client(SetBool, 'inject_fault')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.iter_trigger_sub = self.create_subscription(Int8, '/mission_circuit/record', self.iter_trigger_callback, 1)
        self.timer = self.create_timer(timer_period_sec=5, callback=self.timer_callback)

    def send_request(self, state):
        request = SetBool.Request(data = state)
        self.fault_active = state
        self.future = self.cli.call_async(request)
        self.get_logger().warning(f"Set fault to {state}")
        return self.future.result()
        # return True

    def iter_trigger_callback(self, msg):
        if not self.faulty_runs:
            self.iter_active = False
            return

        self.iter_active = msg.data > -1
        if msg.data == -1:
            self.get_logger().warning("On circuit end. Removing Faults.")
            self.send_request(False)
        return

    def timer_callback(self):
        if not self.faulty_runs:
            return

        if self.iter_active:
            trigger = random.randint(0, 100)
            if trigger > 50 and not self.fault_active:
                self.send_request(True)
            elif trigger <= 50 and self.fault_active:
                self.send_request(False)
            else:
                return
        else:
            return

    def _init_mission_params(self) -> None:
        config_path = get_package_share_directory(
            "px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        for sensor in self.mission_params['sensors']:
            if sensor['fault_active']:
                self.faulty_runs = True

    def _create_directory(self) -> None:
        dir_name = self.get_clock().now().seconds_nanoseconds()[0] / 100
        self.folder_name = f"{os.getcwd()}/records/{str(int(dir_name))}"
        if not os.path.exists(self.folder_name):
            os.makedirs(self.folder_name)
            self.get_logger().warning(
                f"Directory '{self.folder_name}' created successfully.")
        else:
            self.get_logger().info(
                f"Directory '{self.folder_name}' already exists.")

    def _csv_string_to_list(self, csv_string):
        csv_file = StringIO(csv_string)

        csv_reader = csv.reader(csv_file)
        row_list = next(csv_reader, [])

        return row_list


def main(args=None) -> None:
    print('Starting fault_injection_manager node...')
    rclpy.init(args=args)
    try:
        fault_injection_manager = FaultInjectionManager()
        rclpy.spin(fault_injection_manager)
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass


if __name__ == '__main__':
    main()
