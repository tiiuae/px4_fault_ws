#!/usr/bin/env python3

import os
import random
import rclpy
import time
import yaml
import threading
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from ament_index_python.packages import get_package_share_directory


class FaultManager(Node):
    """
    A ROS node for managing faults in a simulated drone environment.
    It injects faults into the system based on predefined parameters and monitors the system's response.
    """

    def __init__(self) -> None:
        super().__init__('fault_manager')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.faulty_sensors = {}
        self.folder_name = ""
        self.faults_active = False
        self.iteration_thread = None
        self.current_interation = 0
        self.stop_thread_event = threading.Event()
        self.out_string = ""
        self.in_iteration = False
        self.fault_window = [1, 5]  # Arbitrary default values

        self.new_inter_sub = self.create_subscription(Int8, '/iteration/current', self._run_iteration, 1)
        self.iter_state = self.create_subscription(String, '/iteration/state', self._iter_state_update, qos)
        self.sim_state = self.create_subscription(String, '/gazebo/state', self._sim_state_update, qos)

        self.param_float_pub = self.create_publisher(String, "/drone_controller/set_param_float", 1)
        self.param_int_pub = self.create_publisher(String, "/drone_controller/set_param_int", 1)

        self._create_directory()
        self._init_mission_params()

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
        Initializes mission parameters from a YAML configuration file.
        """
        config_path = get_package_share_directory(
            "px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        self.fault_window = self.mission_params['time_range_for_faults']

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

    def _activate_faults(self):
        """
        Activates faults based on the current configuration.
        """
        self.faults_active = True
        self.out_string += f"{int(self.get_clock().now().nanoseconds / 1000)},"
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
        """
        Deactivates all active faults.
        """
        self.faults_active = False
        self.out_string += f"{int(self.get_clock().now().nanoseconds / 1000)},"
        self.out_string += "0,"
        for key in list(self.faulty_sensors.keys()):
            sensor = self.faulty_sensors[key]
            for fault_type in list(sensor.keys()):
                if fault_type != 'activator':
                    self.param_float_pub.publish(String(data = f"{sensor[fault_type]['label']}/{0.0}"))
                    self.out_string += f"{0.0},"
            self.param_int_pub.publish(String(data = f"{sensor['activator']}/{0}"))
        self.out_string = self.out_string[:-1] + "\n"

    def _run_iteration(self, msg: Int8):
        """
        Runs an iteration of fault injection based on the provided message.
        """
        if self.iteration_thread is not None and self.iteration_thread.is_alive():
            self.stop_thread_event.set()

        self.current_interation = msg.data
        self.in_iteration = True
        self.out_string = ""
        self.stop_thread_event.clear()
        self.iteration_thread = threading.Thread(target=self._iteration_execution, args=(int(msg.data), ))
        self.iteration_thread.start()

    def _iteration_execution(self, iter_msg: int):
        """
        Executes an iteration of fault injection.
        """
        self.get_logger().info(f"Fault injection in iteration: {iter_msg}")
        while not self.stop_thread_event.is_set():
            # Random sleep interval, you can adjust this as needed
            sleep_time = random.uniform(self.fault_window[0], self.fault_window[1])
            time.sleep(sleep_time)

            # Check if the thread should stop
            if self.stop_thread_event.is_set():
                break

            # Activate or deactivate faults based on current state
            if self.faults_active:
                self._deactivate_faults()
            else:
                self._activate_faults()
        self.get_logger().info("Fault injection halted. Iteration over.")

    def _iter_state_update(self, msg: String):
        """
        Updates the state of an iteration based on the provided message.
        """
        if msg.data == "COMPLETED":
            self.in_iteration = False
            self._deactivate_faults()
            self._dump_data()
            self.stop_thread_event.set()

    def _sim_state_update(self, msg: String):
        """
        Updates the simulation state based on the provided message.
        """
        if self.in_iteration and msg.data != "ACTIVE":
            # self.get_logger().error("Iteration preempted.")
            self.in_iteration = False
            # self._deactivate_faults()
            self._dump_data(preempt = True)

            self.stop_thread_event.set()

    def _dump_data(self, preempt = False):
        """
        Dumps the data of the current iteration to a file.
        """
        self.header = "timestamp,fault_state,"
        for key in list(self.faulty_sensors.keys()):
            sensor = self.faulty_sensors[key]
            for fault_type in list(sensor.keys()):
                if fault_type == 'activator':
                    pass
                else:
                    self.header += f"{sensor[fault_type]['label']},"
        self.out_string = self.header[:-1] + "\n" + self.out_string
        if preempt:
            self.out_string += "PREEMPTED\n"

        if not os.path.exists(f"{self.folder_name}/iteration_{self.current_interation}"):
            os.makedirs(f"{self.folder_name}/iteration_{self.current_interation}")
            self.get_logger().warning(f"Directory '{self.folder_name}/iteration_{self.current_interation}' created successfully.")
        else:
            self.get_logger().info(f"Directory '{self.folder_name}/iteration_{self.current_interation}' already exists.")

        with open(f'{self.folder_name}/iteration_{self.current_interation}/faults.csv', 'w') as f:
            f.write(self.out_string)

        self.out_string = ""
        self.header = ""


def main(args=None) -> None:
    """
    Main function to initialize and run the FaultManager node.
    """
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
