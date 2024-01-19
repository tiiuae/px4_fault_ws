#!/bin/bash

set -e

cd ~/px4_fault_ws
mv build.sh src/build.sh
mv create_session.sh src/create_session.sh
mv kill_session.sh src/kill_session.sh
mv launch_sim.sh src/launch_sim.sh
mv configure.py src/configure.py

cd .src/
git pull

mv build.sh ../build.sh
mv create_session.sh ../create_session.sh
mv kill_session.sh ../kill_session.sh
mv launch_sim.sh ../launch_sim.sh
mv configure.py ../configure.py

cd ..

colcon build

source ~/px4_fault_ws/install/local_setup.bash
