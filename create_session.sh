#!/bin/bash

# Name of the tmux session
SESSION_NAME="px4_sim"

# Create a new detached tmux session
tmux new-session -d -s $SESSION_NAME

# Run your commands in the tmux session
tmux send-keys -t $SESSION_NAME 'cd ~/Documents/PX4-Autopilot/' C-m
tmux send-keys -t $SESSION_NAME "make px4_sitl gz_x500" C-m
# tmux send-keys -t $SESSION_NAME 'HEADLESS=1 make px4_sitl gz_x500' C-m
echo "tmux session $SESSION_NAME started"

