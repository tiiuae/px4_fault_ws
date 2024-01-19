#!/bin/bash

# Source the ros workspace
source install/local_setup.bash

# The DDS Agent needs to be active for the ros2 topics to be visible
# This is activated in a separate tmux session
SESSION_NAME="dds_agent"
tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'MicroXRCEAgent udp4 -p 8888' C-m
echo "tmux session $SESSION_NAME activated."

# Launch sequence for the simulation
echo "Launching mission sequence..."
ros2 run px4_fault_injection drone_controller.py &
sleep 2
ros2 run px4_fault_injection simulation_manager.py &
sleep 10
ros2 run px4_fault_injection drone_state_monitor.py &
ros2 run px4_fault_injection sensor_recorder_unsync.py &
ros2 run px4_fault_injection mission_manager.py &
ros2 run px4_fault_injection iteration_runner.py &
ros2 run px4_fault_injection fault_manager.py &
ros2 run px4_fault_injection data_merger.py
wait

# Close all active tmux sessions when the simulation is terminated.
echo "Shutting down simulation."
tmux kill-session -t $SESSION_NAME
SESSION_NAME="px4_sim"
killall -9 px4
killall -9 ninja
killall -9 make
killall -9 java
PROCESS_NAME="gz sim"
PID=$(ps aux | grep "$PROCESS_NAME" | grep -v grep | awk '{print $2}')
if [ -z "$PID" ]; then
    echo "No process found with name $PROCESS_NAME"
else
    kill $PID
    echo "Process $PROCESS_NAME (PID $PID) has been killed."
fi
tmux kill-session -t $SESSION_NAME
echo "tmux session $SESSION_NAME killed"
