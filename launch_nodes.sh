#!/bin/bash

colcon build --packages-select px4_custom_interfaces px4_fault_injection

source install/local_setup.bash

ros2 run px4_fault_injection data_merger_service.py &

ros2 run px4_fault_injection mission_circuit.py &

# ros2 run px4_fault_injection circuit_recorder.py

ros2 run px4_fault_injection sensor_recorder_unsync.py

wait
