#!/bin/bash

colcon build --packages-select px4_custom_interfaces px4_fault_injection

source install/local_setup.bash

ros2 run px4_fault_injection drone_controller.py &

ros2 run px4_fault_injection simulation_manager.py &

ros2 run px4_fault_injection drone_state_monitor.py

# ros2 run px4_fault_injection iteration_runner.py

wait
