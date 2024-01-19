#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status.

sudo apt update
sudo apt install -y tmux

echo "Pulling the Faulty Autopilot"
cd ~/Documents || exit
git clone --recursive https://github.com/juniorsundar-tii/PX4-Autopilot.git
echo "Building PX4-Autopilot"
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
echo "Build complete"

echo "Setting up Simulation repository"
cd ~/px4_fault_ws || exit
mkdir -p src

mv px4_msgs src/px4_msgs
mv px4_fault_injection src/px4_fault_injection

echo "Building Simulation management"
colcon build

echo "Source the setup script to configure your environment"
echo "source ~/px4_fault_ws/install/local_setup.bash"
