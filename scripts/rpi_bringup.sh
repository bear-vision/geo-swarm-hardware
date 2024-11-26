#!/bin/bash

# Source the local setup script
cd ~/geo-swarm-hardware/ros2-ws
source install/local_setup.bash

# Launch nodes - TODO write launch file

# Call MicroXRCEAgent to set up the serial connection with Pixhawk
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 115200
