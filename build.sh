#!/bin/bash

mkdir src

colcon build

mv px4_custom_interfaces src/px4_custom_interfaces
mv px4_msgs src/px4_msgs
mv px4_fault_injection src/px4_fault_injection

colcon build

source install/local_setup.bash
