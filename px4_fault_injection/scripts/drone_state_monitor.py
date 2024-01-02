#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
import time
from ament_index_python.packages import get_package_share_directory
import yaml


class DroneStateMonitor(Node):
    """
    A ROS node designed to monitor the state of a drone, particularly for detecting failures.
    It subscribes to drone state and simulation state topics, and publishes commands based on the detected conditions.
    """

    def __init__(self):
        """
        Constructor for the DroneStateMonitor class.
        Initializes the ROS node and sets up publishers and subscribers.
        """
        super().__init__('drone_state_monitor')

        # Initial state and parameters
        self.sim_active = False
        self.mission_params = {}
        self.faulty_sensors = {}

        # Setting up QoS profiles for subscriptions
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.drone_failure_detector_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._drone_failure_detector_callback, qos)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sim_state_sub = self.create_subscription(String, '/gazebo/state', self._sim_state_callback, qos)

        # Publishers
        self.drone_state_pub = self.create_publisher(String, "/gazebo/trigger", 10)
        self.param_float_pub = self.create_publisher(String, "/drone_controller/set_param_float", 1)
        self.param_int_pub = self.create_publisher(String, "/drone_controller/set_param_int", 1)

        # Initialize mission parameters
        self._init_mission_params()

    def _init_mission_params(self) -> None:
        """
        Initializes mission parameters by loading them from a YAML configuration file.
        """
        config_path = get_package_share_directory("px4_fault_injection") + "/config/circuit_params.yaml"
        try:
            with open(config_path, 'r') as yaml_file:
                self.mission_params = yaml.safe_load(yaml_file)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error reading YAML file: {e}.")

        # Process sensor fault parameters
        for sensor in self.mission_params['sensors']:
            # Extract sensor details
            module_name = sensor['module_name']
            root = sensor['root']
            active_faults = []

            # Process each fault
            for fault in sensor['faults']:
                if fault['active']:
                    fault_type, label = fault['type'].split('.')

                    # Record active faults
                    if fault_type not in active_faults:
                        active_faults.append(fault_type)

                    # Store faulty sensor details
                    if module_name not in self.faulty_sensors:
                        self.faulty_sensors[module_name] = {}
                        self.faulty_sensors[module_name]['activator'] = root + "_FAULT"
                    if fault_type not in self.faulty_sensors[module_name]:
                        self.faulty_sensors[module_name][fault_type] = {}

                    self.faulty_sensors[module_name][fault_type]['vals'] = fault['vals']
                    self.faulty_sensors[module_name][fault_type]['label'] = root + label

            # Remove sensors without active faults
            if not active_faults and module_name in self.faulty_sensors:
                del self.faulty_sensors[module_name]

    def _deactivate_faults(self):
        """
        Deactivates all active faults.
        """
        self.faults_active = False
        for key in list(self.faulty_sensors.keys()):
            sensor = self.faulty_sensors[key]
            for fault_type in list(sensor.keys()):
                if fault_type != 'activator':
                    self.param_float_pub.publish(String(data=f"{sensor[fault_type]['label']}/{0.0}"))
            self.param_int_pub.publish(String(data=f"{sensor['activator']}/{0}"))

    def _drone_failure_detector_callback(self, msg: VehicleLocalPosition):
        """
        Callback for the drone failure detector.
        Deactivates faults and kills the simulation if certain conditions are met.
        """
        if not self.sim_active:
            return

        # Check position and velocity limits
        pos_limits = self.mission_params['boundaries']
        pos_cond1 = msg.x > pos_limits['east_west']['upper'] or msg.y > pos_limits['north_south']['upper'] or msg.z < pos_limits['altitude']['upper']
        pos_cond2 = msg.x < pos_limits['east_west']['lower'] or msg.y < pos_limits['north_south']['lower']
        vel_cond = abs(msg.vx) > 200 or abs(msg.vy) > 200 or abs(msg.vz) > 200
        acc_cond = abs(msg.ax) > 200 or abs(msg.ay) > 200 or abs(msg.az) > 200

        # Trigger fault deactivation and simulation kill if conditions are met
        if pos_cond1 or pos_cond2 or vel_cond or acc_cond:
            self._deactivate_faults()
            time.sleep(3)
            self.drone_state_pub.publish(String(data="KILL_RE"))
            self.sim_active = False
            time.sleep(3)
            return
        else:
            return

    def _sim_state_callback(self, msg: String):
        """
        Callback for the simulation state.
        Updates the simulation active state based on the received message.
        """
        if msg.data == "ACTIVE":
            self.sim_active = True
        else:
            self.sim_active = False
        return


def main(args=None) -> None:
    """
    Main function to initialize and run the DroneStateMonitor node.
    """
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
