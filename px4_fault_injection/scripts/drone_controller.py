#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.param import ParamError
from mavsdk.offboard import OffboardError, PositionNedYaw
from std_msgs.msg import String, Empty, Float32MultiArray


class DroneController(Node):
    """
    A ROS node for controlling a MAVSDK compatible drone.
    This node subscribes to various topics to control the drone's
    actions such as initialisation, activation, deactivation, moving,
    and setting parameters.
    """

    def __init__(self):
        """
        Constructor for the DroneController class.
        Initialises the ROS node and sets up subscriptions.
        """
        super().__init__('drone_controller')
        self.done = None
        self.loop = asyncio.get_event_loop()

        # Subscriptions
        self.initialise_sub = self.create_subscription(Empty, "/drone_controller/initialise", self.initialise_drone, 10)
        self.activate_sub = self.create_subscription(Empty, "/drone_controller/activate", self.activate_drone, 10)
        self.deactivate_sub = self.create_subscription(Empty, "/drone_controller/deactivate", self.deactivate_drone, 10)
        self.move_sub = self.create_subscription(Float32MultiArray, "/drone_controller/move_drone_NEDY", self.go_to_positionNEDYaw, 10)
        self.param_float_sub = self.create_subscription(String, "/drone_controller/set_param_float", self.set_param_float, 20)
        self.param_int_sub = self.create_subscription(String, "/drone_controller/set_param_int", self.set_param_int, 20)

    def initialise_drone(self, msg):
        """
        Initialises the drone by connecting to it and waiting for it
        to be ready.
        """
        self.loop.run_until_complete(self._initialise_drone())

    async def _initialise_drone(self):
        """
        Asynchronous method to initialise the drone.
        Connects to the drone and checks its state.
        """
        self.drone = System()
        await self.drone.connect(system_address="udp://:14540")

        # Log connection and health status
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Connected to drone!")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position estimate OK")
                break

        self.get_logger().warning("Drone initialised")

    def activate_drone(self, msg):
        """
        Activates the drone, making it ready for offboard control.
        """
        self.loop.run_until_complete(self._activate_drone())

    async def _activate_drone(self):
        """
        Asynchronous method to arm the drone and start offboard mode.
        """
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

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.5, 0.0))
        self.get_logger().warning("Drone activated")

    def deactivate_drone(self, msg):
        """
        Deactivates the drone by stopping the offboard mode and landing.
        """
        self.loop.run_until_complete(self._deactivate_drone())

    async def _deactivate_drone(self):
        """
        Asynchronous method to stop the offboard mode and land the drone.
        """
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

    def go_to_positionNEDYaw(self, msg: Float32MultiArray):
        """
        Moves the drone to a specified position using NED (North, East, Down) coordinates and Yaw.
        """
        self.loop.run_until_complete(self._go_to_positionNEDYaw(msg.data))

    async def _go_to_positionNEDYaw(self, point):
        """
        Asynchronous method to move the drone to the specified NED position and Yaw.
        """
        await self.drone.offboard.set_position_ned(PositionNedYaw(point[0], point[1], point[2], point[3]))

    def set_param_float(self, msg: String):
        """
        Sets a float parameter on the drone.
        """
        string, value = msg.data.split('/')
        self.loop.run_until_complete(self._set_param_float(string, float(value)))
        self.get_logger().error(f"set {string} to {value}")

    async def _set_param_float(self, string, value):
        """
        Asynchronous method to set a float parameter on the drone.
        """
        try:
            await self.drone.param.set_param_float(string, value)
        except ParamError as e:
            self.get_logger().error(f"Param cannot be set at current moment with exception {e}")

    def set_param_int(self, msg: String):
        """
        Sets an integer parameter on the drone.
        """
        string, value = msg.data.split('/')
        self.loop.run_until_complete(self._set_param_int(string, int(value)))
        self.get_logger().error(f"set {string} to {value}")

    async def _set_param_int(self, string, value):
        """
        Asynchronous method to set an integer parameter on the drone.
        """
        try:
            await self.drone.param.set_param_int(string, value)
        except ParamError as e:
            self.get_logger().error(f"Param cannot be set at current moment with exception {e}")


def main(args=None) -> None:
    """
    Main function to initialise and run the ROS node.
    """
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
        print(e)
        pass


if __name__ == '__main__':
    main()
