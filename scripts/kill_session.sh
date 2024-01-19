#!/bin/bash

# Name of the tmux session
SESSION_NAME="px4_sim"

killall -9 px4
killall -9 ninja
killall -9 make
killall -9 java

# Replace 'process_name' with the name of the process you want to kill
PROCESS_NAME="gz sim"

# Find the process ID
PID=$(ps aux | grep "$PROCESS_NAME" | grep -v grep | awk '{print $2}')

# Check if the PID was found
if [ -z "$PID" ]; then
    echo "No process found with name $PROCESS_NAME"
else
    # Kill the process
    kill $PID
    echo "Process $PROCESS_NAME (PID $PID) has been killed."
fi

# Kill the tmux session
tmux kill-session -t $SESSION_NAME

echo "tmux session $SESSION_NAME killed"

