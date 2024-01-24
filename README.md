# PX4 Fault Testing ROS 2 Workspace
ROS 2 Workspace to implement and record faults in PX4
Assumes that you have ROS 2 Humble installed in your system.
If this isn't done, please follow the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Installation

```
mkdir -p ~/px4_fault_ws
git clone --recursive https://github.com/juniorsundar-tii/px4_fault_ws.git ~/px4_fault_ws
cd ~/px4_fault_ws/
chmod +x ./scripts/*.sh
./scripts/build.sh
```

## Execution

```
./scripts/launch_sim.sh
```
