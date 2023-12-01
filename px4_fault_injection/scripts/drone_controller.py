#!/usr/bin/env python3

import asyncio
import csv
import os
import random
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from px4_msgs.msg import *
from std_msgs.msg import String, Empty, Float32MultiArray
from io import StringIO

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        self.drone = System()

        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self._initialise_drone())

        self.activate_sub = self.create_subscription(Empty, "/drone_controller/activate", self.activate_drone, 10)
        self.deactivate_sub = self.create_subscription(Empty, "/drone_controller/deactivate", self.deactivate_drone, 10)
        self.move_sub = self.create_subscription(Float32MultiArray, "/drone_controller/move_drone_NEDY", self.go_to_positionNEDYaw, 10)
        self.param_float_sub = self.create_subscription(String, "/drone_controller/set_param_float", self.set_param_float, 10)
        self.param_int_sub = self.create_subscription(String, "/drone_controller/set_param_int", self.set_param_int, 10)

    async def _initialise_drone(self):
        await self.drone.connect(system_address="udp://:14540")
        
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Connected to drone!")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position estimate OK")
                break

        self.get_logger().warning("Drone initialised")

    def activate_drone(self, msg):
        self.loop.run_until_complete(self._activate_drone())
        return

    async def _activate_drone(self):
        self.get_logger().info("Arming")
        await self.drone.action.arm()
        self.get_logger().info("Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.get_logger().error(f"Starting offboard mode failed with error code: {error._result.result}")
            self.get_logger().error("Disarming")
            await self.drone.action.disarm()
            return
        self.get_logger().warning("Drone activated")

    def deactivate_drone(self, msg):
        self.loop.run_until_complete(self._deactivate_drone())
        return
      
    async def _deactivate_drone(self):
        self.get_logger().info("Stopping offboard")
        try:
            await self.drone.offboard.stop()
        except OffboardError as error:
            self.get_logger().error(f"Stopping offboard mode failed with error code: {error._result.result}")

        await self.drone.action.land()
        await asyncio.sleep(10)
        self.get_logger().info("Disarming")
        await self.drone.action.disarm()
        self.get_logger().warning("Drone deactivated")
        return
    
    def go_to_positionNEDYaw(self, msg: Float32MultiArray):
        self.loop.run_until_complete(self._go_to_positionNEDYaw(msg.data))
        return
    
    async def _go_to_positionNEDYaw(self, point):
        await self.drone.offboard.set_position_ned(PositionNedYaw(point[0], point[1], point[2], point[3]))
        await asyncio.sleep(5)
        return
    
    def set_param_float(self, msg: String):
        string, value = msg.data.split('/')
        asyncio.run(self._set_param_float(string, float(value)))

    async def _set_param_float(self, string, value):
        await self.drone.param.set_param_float(string, value)

    def set_param_int(self, msg: String):
        string, value = msg.data.split('/')
        asyncio.run(self._set_param_float(string, float(value)))

    async def _set_param_int(self, string, value):
        await self.drone.param.set_param_int(string, value)

def main(args=None) -> None:
    print('Starting drone_controller node...')
    rclpy.init(args=args)
    try:
        drone_controller = DroneController()
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    
    try:
        rclpy.shutdown()
    except Exception as e:
        pass


if __name__ == '__main__':
    main()