#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import threading


class IterationRunner(Node):
    """
    A ROS node for running iterations in a simulated drone environment.
    This node manages the execution of waypoint navigation based on received instructions.
    """

    SLEEP_TIME = 5.0  # Constant defining the sleep time between waypoint movements

    def __init__(self) -> None:
        super().__init__('iteration_runner')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Subscribers
        self.waypoint_receiver = self.create_subscription(Float32MultiArray, '/iteration/waypoints', self._run_waypoints, 10)
        self.sim_state_sub = self.create_subscription(String, '/gazebo/state', self._sim_state_callback, qos)

        # Publishers
        self.move_pub = self.create_publisher(Float32MultiArray, "/drone_controller/move_drone_NEDY", 10)
        self.state_pub = self.create_publisher(String, '/iteration/state', qos)

        # Thread management for waypoint execution
        self.waypoint_thread = None
        self.stop_thread_event = threading.Event()

    def _run_waypoints(self, waypoints: Float32MultiArray) -> None:
        """
        Callback function to handle the received waypoints.
        It starts a new thread to execute the waypoints if not already running.
        """
        if self.waypoint_thread is not None and self.waypoint_thread.is_alive():
            self.stop_thread_event.set()

        self.stop_thread_event.clear()
        self.waypoint_thread = threading.Thread(target=self._waypoint_execution, args=(waypoints,))
        self.waypoint_thread.start()

    def _waypoint_execution(self, waypoints: Float32MultiArray):
        """
        Executes waypoint navigation in a separate thread.
        """
        self.state_pub.publish(String(data='STARTED'))

        waypoint_array = np.array(waypoints.data).reshape(-1, 4).tolist()
        for point in waypoint_array:
            if self.stop_thread_event.is_set():
                self.state_pub.publish(String(data='PREEMPTED'))
                return
            else:
                self.get_logger().info(f"Going to: [{round(point[0], 4)}, {round(point[1], 4)}, {round(point[2], 4)}, {round(point[3],4)}]")
                pos = Float32MultiArray()
                pos.data = point

                self.move_pub.publish(pos)
                self.get_clock().sleep_for(Duration(seconds=self.SLEEP_TIME))

        self.state_pub.publish(String(data='COMPLETED'))

    def _sim_state_callback(self, msg: String) -> None:
        """
        Callback for simulation state changes.
        Stops waypoint execution if the simulation becomes inactive.
        """
        if msg.data != 'ACTIVE':
            self.stop_thread_event.set()


def main(args=None) -> None:
    """
    Main function to initialize and run the IterationRunner node.
    """
    print('Starting iteration_runner node...')
    rclpy.init(args=args)
    try:
        iteration_runner = IterationRunner()
        rclpy.spin(iteration_runner)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
