#!/usr/bin/env python3

import csv
import os

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


class SensorRecorder(Node):

    SLEEP_TIME = 5

    def __init__(self) -> None:
        super().__init__('sensor_recorder')
        
        # EXPAND ___________________
        self.sensor_subs = {
            "accelerometer": None,
            "gyroscope": None,
            "magnetometer": None,
            "barometer": None,
        }
        self.sub_callbacks = {
            "accelerometer": self.accel_callback,
            "gyroscope": self.gyro_callback,
            "magnetometer": self.mag_callback,
            "barometer": self.baro_callback,
        }
        self.active_sensors = {
            "accelerometer": False,
            "gyroscope": False,
            "magnetometer": False,
            "barometer": False,
        }
        self.sensor_csvs = {
            "accelerometer": None,
            "gyroscope": None,
            "magnetometer": None,
            "barometer": None,
        }
        self.sensor_msgs = {
            "accelerometer": SensorAccel,
            "gyroscope": SensorGyro,
            "magnetometer": SensorMag,
            "barometer": SensorBaro,
        }
        #_______________________________

        self.folder_name = ""
        self.mission_params = {}
        self.recording = False
        self.prev_record = -1
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._init_mission_params()
        self._create_directory()
        
        self.cli = self.create_client(MergeTarget, 'merge_target')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._create_subscribers()

    def send_request(self, request):
        self.future = self.cli.call_async(request)
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
        for key in list(self.active_sensors.keys()):
            if self.active_sensors[key]:
                self.sensor_csvs[key] = csv.writer(open(f"{self.folder_name}/iteration_{record_id}/{key}.csv", mode='w', newline=''))
                header = []
                empty_msg_dict = message_to_ordereddict(self.sensor_msgs[key]())
                for sub_key in list(empty_msg_dict.keys()):
                    try:
                        for j in range(len(empty_msg_dict[sub_key])):
                            header.append(sub_key+f"_{j}")
                    except TypeError:
                        header.append(sub_key)
                self.sensor_csvs[key].writerow(header)
                self.prev_record = record_id
    
    def _end_record(self, record_id):
        if self.recording:
            request = MergeTarget.Request()
            request.directory = String(data=f"{self.folder_name}/iteration_{self.prev_record}")
            result = self.send_request(request)
            for key in self.active_sensors.keys():
                self.sensor_csvs[key] = None
        else:
            return

    def _csv_string_to_list(self, csv_string):
        csv_file = StringIO(csv_string)

        csv_reader = csv.reader(csv_file)
        row_list = next(csv_reader, [])

        return row_list
        
    def record_update(self, msg: Int8):
        if msg.data < 0:
            self._end_record(msg.data)
            self.recording = False
        else:
            self._start_record(msg.data)
            self.recording = True

    def _create_subscribers(self) -> None:
        self.record_subscriber = self.create_subscription(Int8, "/mission_circuit/record", self.record_update, 10)

        for sensors in self.mission_params['sensors']:
            if sensors["record"]:
                self.active_sensors[sensors['module_name']] = True
                self.sensor_subs[sensors['module_name']] = self.create_subscription(self.sensor_msgs[sensors['module_name']],
                                                                                    sensors["topic_name"],
                                                                                    self.sub_callbacks[sensors['module_name']],
                                                                                    self.qos_profile)

    #____________________________________________
    def accel_callback(self, msg: SensorAccel):
        if not self.active_sensors["accelerometer"]:
            return
        elif not self.recording:
            return
        else:
            row = self._csv_string_to_list(message_to_csv(msg))
            self.sensor_csvs["accelerometer"].writerow(row)
    
    def gyro_callback(self, msg: SensorGyro):
        if not self.active_sensors["gyroscope"]:
            return
        elif not self.recording:
            return
        else:
            row = self._csv_string_to_list(message_to_csv(msg))
            self.sensor_csvs["gyroscope"].writerow(row)
        
    def mag_callback(self, msg: SensorMag):
        if not self.active_sensors["magnetometer"]:
            return
        elif not self.recording:
            return
        else:
            row = self._csv_string_to_list(message_to_csv(msg))
            self.sensor_csvs["magnetometer"].writerow(row)
        
    def baro_callback(self, msg: SensorBaro):
        if not self.active_sensors["barometer"]:
            return
        elif not self.recording:
            return
        else:
            row = self._csv_string_to_list(message_to_csv(msg))
            self.sensor_csvs["barometer"].writerow(row)
    #____________________________________________

def main(args=None) -> None:
    print('Starting sensor_recorder node...')
    rclpy.init(args=args)
    try:
        sensor_recorder = SensorRecorder()
        rclpy.spin(sensor_recorder)
    except KeyboardInterrupt:
        pass
    
    try:
        rclpy.shutdown()
    except Exception as e:
        pass

if __name__ == '__main__':
    main()
