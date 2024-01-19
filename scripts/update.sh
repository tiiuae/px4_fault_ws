#!/bin/bash

set -e

cd ~/px4_fault_ws

mv -r ./src/px4_fault_injection ./px4_fault_injection

git pull

mv -r ./px4_fault_injection ./src/px4_fault_injection

cd ..

colcon build

source ~/px4_fault_ws/install/local_setup.bash
