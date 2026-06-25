#!/bin/bash

echo "Starting MicroXRCEAgent..."

gnome-terminal -- bash -c "
MicroXRCEAgent udp4 -p 8888;
exec bash
"

sleep 2

echo "Starting PX4 Instance 1..."

gnome-terminal -- bash -c "
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE='0,0,0' \
./build/px4_sitl_default/bin/px4 -i 1;
exec bash
"

sleep 3

echo "Starting PX4 Instance 2..."

gnome-terminal -- bash -c "
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE='5,0,0' \
./build/px4_sitl_default/bin/px4 -i 2;
exec bash
"

sleep 3

echo "Starting PX4 Instance 3..."

gnome-terminal -- bash -c "
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE='0,5,0' \
./build/px4_sitl_default/bin/px4 -i 3;
exec bash
"

sleep 10

echo "Starting ROS swarm launch..."

gnome-terminal -- bash -c "
source ~/ros2_ws/install/setup.bash
ros2 launch swarm_formation swarm.launch.py
exec bash
"
