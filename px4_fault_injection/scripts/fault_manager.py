#!/usr/bin/env python3

import csv
import os
from rclpy.qos import QoSProfile, DurabilityPolicy
import random
import rclpy
from rclpy.node import Node
import random
import time
from std_msgs.msg import Int8, String
from std_srvs.srv import SetBool
from io import StringIO
from ament_index_python.packages import get_package_share_directory
import yaml
import threading

class FaultManager(Node):

    def __init__(self) -> None:
        super().__init__('fault_manager')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.faulty_sensors = {}
        self.faults_active = False #! MAKE SURE TO FLIP THIS AT PREEMPT
        self.iteration_thread = None
        self.stop_thread_event = threading.Event()
        self.out_string = ""

        self.new_inter_sub = self.create_subscription(Int8, '/iteration/current', self.run_iteration, 1)
        self.iter_state = self.create_subscription(String, '/iteration/state', self.iter_state_update, qos)
        self.sim_state = self.create_subscription(String, '/gazebo/state', self.sim_state_update, qos)

        self.param_float_pub = self.create_publisher(String, "/drone_controller/set_param_float", 1)
        self.param_int_pub = self.create_publisher(String, "/drone_controller/set_param_int", 1)

        self._init_mission_params()

    def _init_mission_params(self) -> None:
        config_path = get_package_share_directory(
            "px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        for sensor in self.mission_params['sensors']:
            module_name = sensor['module_name']
            root = sensor['root']
            active_faults = []

            for fault in sensor['faults']:
                if fault['active']:
                    fault_type, label = fault['type'].split('.')

                    # Collect all active faults for this sensor
                    if fault_type not in active_faults:
                        active_faults.append(fault_type)

                    # Initialize nested dictionary and list for active faults
                    if module_name not in self.faulty_sensors:
                        self.faulty_sensors[module_name] = {}
                        self.faulty_sensors[module_name]['activator'] = root + "_FAULT"
                    if fault_type not in self.faulty_sensors[module_name]:
                        self.faulty_sensors[module_name][fault_type] = {}

                    # Append the fault values
                    self.faulty_sensors[module_name][fault_type]['vals'] = fault['vals']
                    self.faulty_sensors[module_name][fault_type]['label'] = root + label

            # Remove the module entry if no active faults were found
            if not active_faults and module_name in self.faulty_sensors:
                del self.faulty_sensors[module_name]
        self._activate_faults()
        self._deactivate_faults()
        self._dump_data()
        print(self.out_string)

    def _activate_faults(self):
        self.faults_active = True
        self.out_string += f"{int(self.get_clock().now().nanoseconds / 1000)}"
        self.out_string += "1,"
        for key in list(self.faulty_sensors.keys()):
            sensor = self.faulty_sensors[key]
            for fault_type in list(sensor.keys()):
                if fault_type == 'activator':
                    self.param_int_pub.publish(String(data = f"{sensor[fault_type]}/{1}"))
                else:
                    random_value = random.random() * (sensor[fault_type]['vals'][1] - sensor[fault_type]['vals'][0]) + sensor[fault_type]['vals'][0]
                    self.param_float_pub.publish(String(data = f"{sensor[fault_type]['label']}/{random_value}"))
                    self.out_string += f"{random_value},"
        self.out_string = self.out_string[:-1] + "\n"

    def _deactivate_faults(self):
        self.faults_active = False
        self.out_string += f"{int(self.get_clock().now().nanoseconds / 1000)}"
        self.out_string += "0,"
        for key in list(self.faulty_sensors.keys()):
            sensor = self.faulty_sensors[key]
            for fault_type in list(sensor.keys()):
                if fault_type != 'activator':
                    self.param_float_pub.publish(String(data = f"{sensor[fault_type]['label']}/{0.0}"))
                    self.out_string += f"{0.0},"
            self.param_int_pub.publish(String(data = f"{sensor['activator']}/{0}"))
        self.out_string = self.out_string[:-1] + "\n"

    def run_iteration(self, msg: Int8):
        if self.iteration_thread is not None and self.iteration_thread.is_alive():
            self.stop_thread_event.set()

        self.stop_thread_event.clear()
        self.iteration_thread = threading.Thread(target=self._iteration_execution)
        self.iteration_thread.start()

    #! TESTING THIS FUNCTION OUT
    def _iteration_execution(self):
        print("in thread")
        while not self.stop_thread_event.is_set():
            # Random sleep interval, you can adjust this as needed
            sleep_time = random.uniform(1, 5)  # Random sleep time between 1 and 5 seconds
            time.sleep(sleep_time)

            # Check if the thread should stop
            if self.stop_thread_event.is_set():
                break

            # Activate or deactivate faults based on current state
            if self.faults_active:
                self._deactivate_faults()
            else:
                self._activate_faults()
        print("out of thread")

    def iter_state_update(self, msg: String):
        if msg.data == "COMPLETED":
            self._deactivate_faults()
            self._dump_data()
            self.stop_thread_event.set()

    def sim_state_update(self, msg: String):
        if msg.data != "ACTIVE":
            self._deactivate_faults()
            self._dump_data()
            self.stop_thread_event.set()

    def _dump_data(self):
        self.header = "timestamp,fault_state,"
        for key in list(self.faulty_sensors.keys()):
            sensor = self.faulty_sensors[key]
            for fault_type in list(sensor.keys()):
                if fault_type == 'activator':
                    pass
                else:
                    self.header += f"{sensor[fault_type]['label']},"
        self.out_string = self.header[:-1] + "\n" + self.out_string


def main(args=None) -> None:
    print('Starting fault_manager node...')
    rclpy.init(args=args)
    try:
        fault_manager = FaultManager()
        rclpy.spin(fault_manager)
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass


if __name__ == '__main__':
    main()
