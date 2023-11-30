#!/usr/bin/env python3

import asyncio
import random
import yaml

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

        self.srv = self.create_service(
            SetBool, 'inject_fault', self.fault_callback)

    def fault_callback(self, request, response):
        response.success = False

        config_path = get_package_share_directory("px4_fault_injection") +\
            "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        if request.data:
            asyncio.run(self.activate_faults(mission_params))
        else:
            asyncio.run(self.deactivate_faults(mission_params))
        response.success = True
        return response

    async def activate_faults(self, mission_params):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.get_logger().info("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Connected to drone!")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position estimate OK")
                break

        for sensor in mission_params['sensors']:
            if sensor['fault_active']:
                await drone.param.set_param_int(self.sensor_fault_switch[sensor['module_name']] + "FAULT",
                                          1)
                for key in list(sensor['fault_vals'].keys()):
                    random_value = random.random() * \
                        (sensor['fault_vals'][key][1] - sensor['fault_vals']
                         [key][0]) + sensor['fault_vals'][key][0]
                    await drone.param.set_param_float(self.sensor_fault_switch[sensor['module_name']] +
                                              self.sensor_fault_type[key],
                                              random_value)
        return

    async def deactivate_faults(self, mission_params):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.get_logger().info("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Connected to drone!")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position estimate OK")
                break

        for sensor in mission_params['sensors']:
            for key in list(sensor['fault_vals'].keys()):
                await drone.param.set_param_float(self.sensor_fault_switch[sensor['module_name']] +
                                          self.sensor_fault_type[key],
                                          0)
            await drone.param.set_param_int(self.sensor_fault_switch[sensor['module_name']] + "FAULT",
                                            0)
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
