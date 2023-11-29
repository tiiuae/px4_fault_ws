#!/usr/bin/env python3

import csv
import importlib
import os

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rosidl_runtime_py import message_to_ordereddict, message_to_csv
from px4_msgs.msg import *
from std_msgs.msg import Int8, String
from io import StringIO
from ament_index_python.packages import get_package_share_directory
import yaml
from px4_custom_interfaces.srv import MergeTarget


class CircuitRecorder(Node):

    SLEEP_TIME = 5

    def __init__(self) -> None:
        super().__init__('circuit_recorder')
        
        self.folder_name = ""
        self.subscribers = []
        self.msg_types = []
        self.active_sensors = []
        self.ts = None
        self.mission_params = {}
        self.data_csvs = []
        self.recording = False
        self.prev_record = -1

        self.cli = self.create_client(MergeTarget, 'merge_target')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._init_mission_params()
        self._create_directory()
        self._create_subscribers()

    def send_request(self, request):
        self.future = self.cli.call_async(request)
        # rclpy.spin_until_future_complete(self, self.future, timeout_sec=2.0)
        return self.future.result()

    def _init_mission_params(self) -> None:
        config_path = get_package_share_directory("px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

    def _create_directory(self) -> None:
        self.folder_name = f"{os.getcwd()}/records/{str(int(self.get_clock().now().seconds_nanoseconds()[0]/100))}"
        if not os.path.exists(self.folder_name):
            os.makedirs(self.folder_name)
            self.get_logger().warning(f"Directory '{self.folder_name}' created successfully.")
        else:
            self.get_logger().info(f"Directory '{self.folder_name}' already exists.")
    
    def _start_record(self, record_id):
        for i in range(len(self.subscribers)):
            self.data_csvs.append(csv.writer(open(f"{self.folder_name}/iteration_{record_id}/{self.active_sensors[i]}.csv", mode='w', newline='')))
            header = []
            empty_msg_dict = message_to_ordereddict(self.msg_types[i]())
            for key in list(empty_msg_dict.keys()):
                try:
                    for j in range(len(empty_msg_dict[key])):
                        header.append(key+f"_{j}")
                except TypeError:
                    header.append(key)
            self.data_csvs[i].writerow(header)
            self.prev_record = record_id
    
    def _end_record(self, record_id):
        if self.recording:
            request = MergeTarget.Request()
            request.directory = String(data=f"{self.folder_name}/iteration_{self.prev_record}")
            result = self.send_request(request)
            self.data_csvs.clear()
        else:
            return

    def _csv_string_to_list(self, csv_string):
        csv_file = StringIO(csv_string)

        csv_reader = csv.reader(csv_file)
        row_list = next(csv_reader, [])

        return row_list
    
    def callback(self, *args):
        if self.recording:
            for i in range(len(args)):
                row = self._csv_string_to_list(message_to_csv(args[i]))
                self.data_csvs[i].writerow(row)
        else:
            return
        
    def record_update(self, msg: Int8):
        if msg.data < 0:
            self._end_record(msg.data)
            self.recording = False
        else:
            self._start_record(msg.data)
            self.recording = True

    def _create_subscribers(self) -> None:
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscribers = []
        self.msg_types = []
        self.record_subscriber = self.create_subscription(Int8, "/mission_circuit/record", self.record_update, 10)

        for sensors in self.mission_params['sensors']:
            if sensors["record"]:
                self.active_sensors.append(sensors['module_name'])
                module_name, class_name = sensors["data_type"].rsplit('/', 1)
                module = importlib.import_module(module_name)
                MessageClass = getattr(module, class_name)
                self.msg_types.append(MessageClass)
                self.subscribers.append(
                    message_filters.Subscriber(self, MessageClass, sensors["topic_name"], qos_profile=qos_profile))

        self.ts = message_filters.ApproximateTimeSynchronizer(self.subscribers, 10, slop=10, allow_headerless=True)
        self.ts.registerCallback(self.callback)

def main(args=None) -> None:
    print('Starting circuit_recorder node...')
    rclpy.init(args=args)
    try:
        circuit_recorder = CircuitRecorder()
        rclpy.spin(circuit_recorder)
    except KeyboardInterrupt:
        pass
    
    try:
        rclpy.shutdown()
    except Exception as e:
        pass

if __name__ == '__main__':
    main()
