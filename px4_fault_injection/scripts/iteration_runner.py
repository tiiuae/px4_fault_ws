#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import threading


class IterationRunner(Node):

    SLEEP_TIME = 5.0

    def __init__(self) -> None:
        super().__init__('iteration_runner')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_receiver = self.create_subscription(Float32MultiArray, '/iteration/waypoints', self.run_waypoints, 10)
        self.move_pub = self.create_publisher(Float32MultiArray, "/drone_controller/move_drone_NEDY", 10)
        self.sim_state = False
        self.sim_state_sub = self.create_subscription(String, '/gazebo/state', self.sim_state_callback, qos)

        self.state_pub = self.create_publisher(String, '/iteration/state', qos)
        self.waypoint_thread = None
        self.stop_thread_event = threading.Event()

    def run_waypoints(self, waypoints: Float32MultiArray) -> None:
        if self.waypoint_thread is not None and self.waypoint_thread.is_alive():
            # If the thread is already running, signal it to stop
            self.stop_thread_event.set()

        # Reset the event and start a new thread
        self.stop_thread_event.clear()
        self.waypoint_thread = threading.Thread(target=self._waypoint_execution, args=(waypoints,))
        self.waypoint_thread.start()

    def _waypoint_execution(self, waypoints: Float32MultiArray):
        # This is the method that runs in the thread
        self.state_pub.publish(String(data='STARTED'))

        waypoint_array = np.array(waypoints.data).reshape(-1, 4).tolist()
        for point in waypoint_array:
            # Check the event to see if we need to stop early
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
        return

    def sim_state_callback(self, msg: String) -> None:
        if msg.data != 'ACTIVE':
            # If the simulation is not active, signal the waypoint thread to stop
            self.stop_thread_event.set()


def main(args=None) -> None:
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
