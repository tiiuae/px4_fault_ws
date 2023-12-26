#!/bin/bash

colcon build --packages-select px4_custom_interfaces px4_fault_injection

source install/local_setup.bash

ros2 run px4_fault_injection drone_controller.py &

sleep 2

ros2 run px4_fault_injection simulation_manager.py &

sleep 10

ros2 run px4_fault_injection drone_state_monitor.py &

ros2 run px4_fault_injection sensor_recorder_unsync.py &

ros2 run px4_fault_injection mission_manager.py &

ros2 run px4_fault_injection iteration_runner.py &

ros2 run px4_fault_injection fault_manager.py

wait
