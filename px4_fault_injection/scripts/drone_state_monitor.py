#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from px4_msgs.msg import FailureDetectorStatus, VehicleLocalPosition
import numpy as np
import subprocess
import time
import os
import threading

class DroneStateMonitor(Node):
    def __init__(self):
        super().__init__('drone_state_monitor')

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sim_active = False

        # self.drone_failure_detector_sub = self.create_subscription(FailureDetectorStatus, "/fmu/out/failure_detector_status", self.drone_failure_detector_callback, self.qos_profile)
        self.drone_failure_detector_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.drone_failure_detector_callback, self.qos_profile)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription1 = self.create_subscription(
            String,
            '/gazebo/state',
            self.listener_callback1,
            qos)
        
        self.drone_state_pub = self.create_publisher(String, "/gazebo/trigger", 10)
        self.cli = self.create_client(SetBool, 'inject_fault')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, state):
        request = SetBool.Request(data = state)
        self.fault_active = state
        self.future = self.cli.call_async(request)
        self.get_logger().warning(f"Set fault to {state}")
        return self.future.result()
        # return True

    # def drone_failure_detector_callback(self, msg: FailureDetectorStatus):
    #     cond1 = msg.fd_alt or msg.fd_arm_escs or msg.fd_battery or msg.fd_pitch
    #     cond2 = msg.fd_ext or msg.fd_imbalanced_prop or msg.fd_motor or msg.fd_roll
    #     if (cond1 or cond2) and self.sim_active:
    #         self.send_request(False)
    #         time.sleep(3)
    #         self.drone_state_pub.publish(String(data="KILL_RE"))
    #         return
    #     else:
    #         return

    def drone_failure_detector_callback(self, msg: VehicleLocalPosition):
        if not self.sim_active:
            return
        
        pos_cond = abs(msg.x) > 200 or abs(msg.y) > 200 or abs(msg.z) > 200
        vel_cond = abs(msg.vx) > 200 or abs(msg.vy) > 200 or abs(msg.vz) > 200
        acc_cond = abs(msg.ax) > 200 or abs(msg.ay) > 200 or abs(msg.az) > 200

        if pos_cond or vel_cond or acc_cond:
            self.send_request(False)
            time.sleep(3)
            self.drone_state_pub.publish(String(data="KILL_RE"))
            time.sleep(3)
            return
        else:
            return
        
    def listener_callback1(self, msg: String):
        if msg.data == "ACTIVE":
            self.sim_active = True
        else:
            self.sim_active = False
        return
    
def main(args=None) -> None:
    print('Starting drone_state_monitor node...')
    rclpy.init(args=args)
    try:
        drone_state_monitor = DroneStateMonitor()
        rclpy.spin(drone_state_monitor)
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass


if __name__ == '__main__':
    main()