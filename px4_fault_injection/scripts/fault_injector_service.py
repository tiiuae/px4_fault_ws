#!/usr/bin/env python3

import random
import yaml

from std_msgs.msg import String
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from mavsdk import System
from px4_msgs.msg import *


class FaultInjectorService(Node):

    SLEEP_TIME = 5

    def __init__(self):
        super().__init__('fault_injector_service')
        self.sensor_fault_switch = {
            "accelerometer": "SENS_ACCEL_",
            "gyroscope": "SENS_GYRO_",
            "magnetometer": "SENS_MAG_",
            "barometer": "SENS_BARO_"
        }
        self.sensor_fault_type = {
            "gaussian_noise": "NOISE",
            "bias_shift": "SHIF",
            "bias_scale": "SCAL",
            "drift": "DRIFT"
        }
        config_path = get_package_share_directory("px4_fault_injection") +\
            "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        self.srv = self.create_service(
            SetBool, 'inject_fault', self.fault_callback)
        self.param_float_pub = self.create_publisher(String, "/drone_controller/set_param_float", 1)
        self.param_int_pub = self.create_publisher(String, "/drone_controller/set_param_int", 1)

    def fault_callback(self, request, response):
        response.success = False
        if request.data:
            self.activate_faults()
        else:
            self.deactivate_faults()
        response.success = True
        return response

    def activate_faults(self):
        for sensor in self.mission_params['sensors']:
            if sensor['fault_active']:
                sending = f"{self.sensor_fault_switch[sensor['module_name']]}FAULT/{1}"
                self.param_int_pub.publish(String(data=sending))

                for key in list(sensor['fault_vals'].keys()):
                    random_value = random.random() * \
                        (sensor['fault_vals'][key][1] - sensor['fault_vals']
                         [key][0]) + sensor['fault_vals'][key][0]
                    sending = f"{self.sensor_fault_switch[sensor['module_name']]}{self.sensor_fault_type[key]}/{random_value}"
                    self.param_float_pub.publish(String(data=sending))
        return

    def deactivate_faults(self):
        for sensor in self.mission_params['sensors']:
            if sensor['fault_active']:
                for key in list(sensor['fault_vals'].keys()):
                    sending = f"{self.sensor_fault_switch[sensor['module_name']]}{self.sensor_fault_type[key]}/{0}"
                    self.param_float_pub.publish(String(data=sending))

                sending = f"{self.sensor_fault_switch[sensor['module_name']]}FAULT/{0}"
                self.param_int_pub.publish(String(data=sending))

        return


def main(args=None) -> None:
    print('Starting fault_injector_service node...')
    rclpy.init(args=args)
    try:
        fault_injector_service = FaultInjectorService()
        rclpy.spin(fault_injector_service)
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        pass


if __name__ == '__main__':
    main()
